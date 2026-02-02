import React, { useState, useEffect, useRef } from 'react';
import * as THREE from 'three';
// import ROSLIB from 'roslib'; // Odkomentuj po instalacji: npm install roslib

const ROSPointCloudViewer = () => {
  const [connected, setConnected] = useState(false);
  const [pointCount, setPointCount] = useState(0);
  const [rosUrl, setRosUrl] = useState('ws://localhost:9090');
  const [topic, setTopic] = useState('/point_cloud_1');
  const [error, setError] = useState('');
  const [stats, setStats] = useState({ min: null, max: null, avg: null });
  const [useSimulation, setUseSimulation] = useState(true); // Przełącznik trybu
  
  const mountRef = useRef(null);
  const sceneRef = useRef(null);
  const cameraRef = useRef(null);
  const rendererRef = useRef(null);
  const pointCloudRef = useRef(null);
  const rosRef = useRef(null);
  const animationIdRef = useRef(null);

  // Inicjalizacja sceny Three.js
  useEffect(() => {
    if (!mountRef.current) return;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x1a1a2e);
    sceneRef.current = scene;

    const camera = new THREE.PerspectiveCamera(
      75,
      mountRef.current.clientWidth / mountRef.current.clientHeight,
      0.1,
      1000
    );
    camera.position.set(5, 5, 5);
    camera.lookAt(0, 0, 0);
    cameraRef.current = camera;

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(mountRef.current.clientWidth, mountRef.current.clientHeight);
    mountRef.current.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);
    
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(10, 10, 10);
    scene.add(directionalLight);

    const gridHelper = new THREE.GridHelper(10, 10, 0x444444, 0x222222);
    scene.add(gridHelper);

    const axesHelper = new THREE.AxesHelper(5);
    scene.add(axesHelper);

    const geometry = new THREE.BufferGeometry();
    const material = new THREE.PointsMaterial({
      size: 0.05,
      vertexColors: true,
      sizeAttenuation: true
    });
    const points = new THREE.Points(geometry, material);
    scene.add(points);
    pointCloudRef.current = points;

    const handleResize = () => {
      if (!mountRef.current) return;
      const width = mountRef.current.clientWidth;
      const height = mountRef.current.clientHeight;
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
      renderer.setSize(width, height);
    };
    window.addEventListener('resize', handleResize);

    let angle = 0;
    const animate = () => {
      animationIdRef.current = requestAnimationFrame(animate);
      
      angle += 0.005;
      camera.position.x = Math.cos(angle) * 8;
      camera.position.z = Math.sin(angle) * 8;
      camera.position.y = 5;
      camera.lookAt(0, 0, 0);
      
      renderer.render(scene, camera);
    };
    animate();

    return () => {
      window.removeEventListener('resize', handleResize);
      if (animationIdRef.current) {
        cancelAnimationFrame(animationIdRef.current);
      }
      if (mountRef.current && renderer.domElement) {
        mountRef.current.removeChild(renderer.domElement);
      }
      renderer.dispose();
    };
  }, []);

  // Przetwarzanie prawdziwych danych PointCloud2 z ROS
  const processPointCloud = (message) => {
    try {
      const points = [];
      const colors = [];
      
      // Dekoduj binarne dane PointCloud2
      const data = new Uint8Array(message.data);
      const pointStep = message.point_step;
      const numPoints = message.width * message.height;
      
      // Standardowe offsety dla XYZ w PointCloud2
      const xOffset = 0;
      const yOffset = 4;
      const zOffset = 8;
      
      let minDist = Infinity;
      let maxDist = -Infinity;
      let sumDist = 0;
      let validPoints = 0;
      
      for (let i = 0; i < numPoints; i++) {
        const offset = i * pointStep;
        
        // Odczytaj współrzędne Float32 (little-endian)
        const x = new DataView(data.buffer).getFloat32(offset + xOffset, true);
        const y = new DataView(data.buffer).getFloat32(offset + yOffset, true);
        const z = new DataView(data.buffer).getFloat32(offset + zOffset, true);
        
        // Pomiń nieprawidłowe punkty
        if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
        
        points.push(x, y, z);
        validPoints++;
        
        const dist = Math.sqrt(x*x + y*y + z*z);
        minDist = Math.min(minDist, dist);
        maxDist = Math.max(maxDist, dist);
        sumDist += dist;
        
        // Kolorowanie na podstawie wysokości (oś Z)
        const normalizedZ = Math.max(0, Math.min(1, (z + 2) / 4));
        colors.push(
          1 - normalizedZ,
          normalizedZ * 0.5,
          normalizedZ
        );
      }
      
      if (pointCloudRef.current && points.length > 0) {
        const positions = new Float32Array(points);
        const colorArray = new Float32Array(colors);
        
        pointCloudRef.current.geometry.setAttribute(
          'position',
          new THREE.BufferAttribute(positions, 3)
        );
        pointCloudRef.current.geometry.setAttribute(
          'color',
          new THREE.BufferAttribute(colorArray, 3)
        );
        
        pointCloudRef.current.geometry.computeBoundingSphere();
        pointCloudRef.current.geometry.attributes.position.needsUpdate = true;
        pointCloudRef.current.geometry.attributes.color.needsUpdate = true;
      }
      
      setPointCount(validPoints);
      setStats({
        min: minDist === Infinity ? '0.00' : minDist.toFixed(2),
        max: maxDist === -Infinity ? '0.00' : maxDist.toFixed(2),
        avg: validPoints > 0 ? (sumDist / validPoints).toFixed(2) : '0.00'
      });
      
    } catch (err) {
      console.error('❌ Błąd przetwarzania chmury punktów:', err);
      setError('Błąd przetwarzania danych: ' + err.message);
    }
  };

  // Połączenie z prawdziwym ROS
  const connectToRealROS = () => {
    setError('');
    
    // Sprawdź czy ROSLIB jest dostępny
    if (typeof window.ROSLIB === 'undefined') {
      setError('⚠️ Biblioteka ROSLIB nie jest załadowana. Zainstaluj: npm install roslib');
      return;
    }
    
    try {
      const ros = new window.ROSLIB.Ros({
        url: rosUrl
      });

      ros.on('connection', () => {
        console.log('✅ Połączono z ROS!');
        setConnected(true);
        setError('');
        
        const listener = new window.ROSLIB.Topic({
          ros: ros,
          name: topic,
          messageType: 'sensor_msgs/PointCloud2'
        });

        listener.subscribe((message) => {
          console.log('📊 Otrzymano chmurę punktów');
          processPointCloud(message);
        });

        rosRef.current = { ros, listener };
      });

      ros.on('error', (error) => {
        console.error('❌ Błąd ROS:', error);
        setError('Błąd połączenia z ROS: ' + error);
        setConnected(false);
      });

      ros.on('close', () => {
        console.log('🔌 Rozłączono z ROS');
        setConnected(false);
      });

    } catch (err) {
      setError('Błąd połączenia: ' + err.message);
      setConnected(false);
    }
  };

  // Symulacja danych (dla testów)
  const generateSimulatedPointCloud = () => {
    const numPoints = 500 + Math.floor(Math.random() * 500);
    const positions = new Float32Array(numPoints * 3);
    const colors = new Float32Array(numPoints * 3);
    
    let minDist = Infinity;
    let maxDist = -Infinity;
    let sumDist = 0;

    for (let i = 0; i < numPoints; i++) {
      const x = (Math.random() - 0.5) * 6;
      const z = (Math.random() - 0.5) * 6;
      const y = Math.sin(x) * Math.cos(z) * 0.5 + (Math.random() - 0.5) * 0.2;
      
      positions[i * 3] = x;
      positions[i * 3 + 1] = y;
      positions[i * 3 + 2] = z;
      
      const normalizedY = (y + 1) / 2;
      colors[i * 3] = 1 - normalizedY;
      colors[i * 3 + 1] = normalizedY * 0.5;
      colors[i * 3 + 2] = normalizedY;
      
      const dist = Math.sqrt(x*x + y*y + z*z);
      minDist = Math.min(minDist, dist);
      maxDist = Math.max(maxDist, dist);
      sumDist += dist;
    }

    if (pointCloudRef.current) {
      pointCloudRef.current.geometry.setAttribute(
        'position',
        new THREE.BufferAttribute(positions, 3)
      );
      pointCloudRef.current.geometry.setAttribute(
        'color',
        new THREE.BufferAttribute(colors, 3)
      );
      pointCloudRef.current.geometry.attributes.position.needsUpdate = true;
      pointCloudRef.current.geometry.attributes.color.needsUpdate = true;
    }

    setPointCount(numPoints);
    setStats({
      min: minDist.toFixed(2),
      max: maxDist.toFixed(2),
      avg: (sumDist / numPoints).toFixed(2)
    });
  };

  const connectToSimulation = () => {
    setConnected(true);
    const interval = setInterval(() => {
      generateSimulatedPointCloud();
    }, 100);
    rosRef.current = { interval };
  };

  const connectToROS = () => {
    if (useSimulation) {
      connectToSimulation();
    } else {
      connectToRealROS();
    }
  };

  const disconnectFromROS = () => {
    if (rosRef.current?.interval) {
      clearInterval(rosRef.current.interval);
    }
    if (rosRef.current?.listener) {
      rosRef.current.listener.unsubscribe();
    }
    if (rosRef.current?.ros) {
      rosRef.current.ros.close();
    }
    setConnected(false);
    setPointCount(0);
  };

  return (
    <div className="w-full h-screen bg-gray-900 text-white flex flex-col">
      {/* Panel kontrolny */}
      <div className="bg-gray-800 p-4 shadow-lg">
        <h1 className="text-2xl font-bold mb-4 text-blue-400">
          🎯 ROS Point Cloud Viewer
        </h1>
        
        {/* Przełącznik trybu */}
        <div className="mb-4 p-3 bg-gray-700 rounded">
          <label className="flex items-center cursor-pointer">
            <input
              type="checkbox"
              checked={useSimulation}
              onChange={(e) => setUseSimulation(e.target.checked)}
              disabled={connected}
              className="mr-3 w-5 h-5"
            />
            <span className="text-sm">
              {useSimulation ? '🎮 Tryb symulacji (testowy)' : '🤖 Tryb ROS (prawdziwe dane)'}
            </span>
          </label>
          <p className="text-xs text-gray-400 mt-1 ml-8">
            {useSimulation 
              ? 'Używa wygenerowanych danych do testów' 
              : 'Wymaga rosbridge_server i biblioteki roslib'}
          </p>
        </div>

        <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
          <div>
            <label className="block text-sm font-medium mb-1">ROS Bridge URL</label>
            <input
              type="text"
              value={rosUrl}
              onChange={(e) => setRosUrl(e.target.value)}
              disabled={connected || useSimulation}
              className="w-full px-3 py-2 bg-gray-700 border border-gray-600 rounded text-white disabled:opacity-50"
              placeholder="ws://localhost:9090"
            />
          </div>
          
          <div>
            <label className="block text-sm font-medium mb-1">Topic</label>
            <input
              type="text"
              value={topic}
              onChange={(e) => setTopic(e.target.value)}
              disabled={connected || useSimulation}
              className="w-full px-3 py-2 bg-gray-700 border border-gray-600 rounded text-white disabled:opacity-50"
              placeholder="/point_cloud_1"
            />
          </div>
          
          <div className="flex items-end">
            <button
              onClick={connected ? disconnectFromROS : connectToROS}
              className={`w-full px-4 py-2 rounded font-medium transition-colors ${
                connected
                  ? 'bg-red-600 hover:bg-red-700'
                  : 'bg-green-600 hover:bg-green-700'
              }`}
            >
              {connected ? '🔴 Rozłącz' : '🟢 Połącz'}
            </button>
          </div>
        </div>

        {error && (
          <div className="mt-3 p-3 bg-red-900 border border-red-700 rounded text-red-200">
            ⚠️ {error}
          </div>
        )}

        {/* Statystyki */}
        {connected && (
          <div className="mt-4 grid grid-cols-2 md:grid-cols-4 gap-3">
            <div className="bg-gray-700 p-3 rounded">
              <div className="text-xs text-gray-400">Status</div>
              <div className="text-lg font-bold text-green-400">
                {useSimulation ? 'Symulacja' : 'Połączono'}
              </div>
            </div>
            <div className="bg-gray-700 p-3 rounded">
              <div className="text-xs text-gray-400">Punkty</div>
              <div className="text-lg font-bold text-blue-400">{pointCount}</div>
            </div>
            <div className="bg-gray-700 p-3 rounded">
              <div className="text-xs text-gray-400">Min/Max odl.</div>
              <div className="text-lg font-bold text-purple-400">
                {stats.min} / {stats.max}
              </div>
            </div>
            <div className="bg-gray-700 p-3 rounded">
              <div className="text-xs text-gray-400">Śr. odległość</div>
              <div className="text-lg font-bold text-yellow-400">{stats.avg}</div>
            </div>
          </div>
        )}
      </div>

      {/* Obszar wizualizacji 3D */}
      <div className="flex-1 relative">
        <div ref={mountRef} className="w-full h-full" />
        
        {/* Legenda */}
        <div className="absolute bottom-4 left-4 bg-gray-800 bg-opacity-90 p-4 rounded shadow-lg">
          <h3 className="font-bold mb-2">📊 Legenda</h3>
          <div className="space-y-1 text-sm">
            <div className="flex items-center gap-2">
              <div className="w-4 h-4 bg-red-500"></div>
              <span>Nisko (czerwony)</span>
            </div>
            <div className="flex items-center gap-2">
              <div className="w-4 h-4 bg-blue-500"></div>
              <span>Wysoko (niebieski)</span>
            </div>
          </div>
          <div className="mt-3 pt-3 border-t border-gray-600 text-xs text-gray-400">
            <div>🔵 Niebieski = oś X</div>
            <div>🟢 Zielony = oś Y</div>
            <div>🔴 Czerwony = oś Z</div>
          </div>
        </div>

        {/* Instrukcje */}
        {!connected && (
          <div className="absolute top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2 text-center">
            <div className="bg-gray-800 bg-opacity-90 p-8 rounded-lg shadow-2xl max-w-md">
              <div className="text-6xl mb-4">🎮</div>
              <h2 className="text-2xl font-bold mb-2">Witaj w Point Cloud Viewer!</h2>
              <p className="text-gray-400 mb-4">
                Wybierz tryb i podłącz się do wizualizacji 3D
              </p>
              <div className="text-sm text-gray-500 text-left">
                <p className="font-bold mb-2">Tryb symulacji:</p>
                <p>• Kliknij "Połącz" aby zobaczyć demo</p>
                
                <p className="font-bold mt-4 mb-2">Tryb ROS:</p>
                <p>1. Uruchom: roslaunch rosbridge_server rosbridge_websocket.launch</p>
                <p>2. Wprowadź URL i topic</p>
                <p>3. Kliknij "Połącz"</p>
              </div>
            </div>
          </div>
        )}
      </div>

      {/* Informacje techniczne */}
      <div className="bg-gray-800 px-4 py-2 text-xs text-gray-400 flex justify-between">
        <span>💡 Kamera rotuje automatycznie wokół sceny</span>
        <span>
          {useSimulation ? '🎮 Tryb symulacji' : `🤖 Topic: ${topic}`}
        </span>
      </div>
    </div>
  );
};

export default ROSPointCloudViewer;
