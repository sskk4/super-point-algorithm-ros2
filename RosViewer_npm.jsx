import React, { useState, useEffect, useRef } from 'react';
import * as THREE from 'three';
import ROSLIB from 'roslib';  // ← LOKALNY IMPORT!

const RosViewer = () => {
  const [connected, setConnected] = useState(false);
  const [pointCount, setPointCount] = useState(0);
  const [rosUrl, setRosUrl] = useState('ws://192.168.1.100:9090');
  const [error, setError] = useState('');
  const [imageData, setImageData] = useState(null);
  
  // Nowe stany dla topiców
  const [availableTopics, setAvailableTopics] = useState([]);
  const [selectedPointCloudTopic, setSelectedPointCloudTopic] = useState('/point_cloud_1');
  const [selectedImageTopic, setSelectedImageTopic] = useState('/camera/image_raw');
  const [loadingTopics, setLoadingTopics] = useState(false);
  
  const mountRef = useRef(null);
  const canvasRef = useRef(null);
  const sceneRef = useRef(null);
  const cameraRef = useRef(null);
  const rendererRef = useRef(null);
  const pointCloudRef = useRef(null);
  const rosRef = useRef(null);

  // Style inline (bez Tailwind!)
  const styles = {
    container: {
      display: 'flex',
      flexDirection: 'column',
      height: '100vh',
      background: '#1a1a2e',
      color: 'white',
      fontFamily: 'Arial, sans-serif'
    },
    header: {
      background: '#16213e',
      padding: '20px',
      boxShadow: '0 2px 10px rgba(0,0,0,0.5)'
    },
    title: {
      color: '#4a9eff',
      marginBottom: '15px',
      fontSize: '24px'
    },
    controls: {
      display: 'grid',
      gridTemplateColumns: '2fr 1fr',
      gap: '15px',
      marginBottom: '15px'
    },
    topicControls: {
      display: 'grid',
      gridTemplateColumns: '1fr 1fr',
      gap: '15px',
      marginBottom: '15px'
    },
    input: {
      padding: '10px',
      borderRadius: '5px',
      border: '1px solid #444',
      background: '#0f3460',
      color: 'white',
      fontSize: '14px'
    },
    select: {
      padding: '10px',
      borderRadius: '5px',
      border: '1px solid #444',
      background: '#0f3460',
      color: 'white',
      fontSize: '14px',
      cursor: 'pointer'
    },
    button: {
      padding: '10px 20px',
      borderRadius: '5px',
      border: 'none',
      fontSize: '14px',
      fontWeight: 'bold',
      cursor: 'pointer',
      transition: 'all 0.3s'
    },
    buttonConnect: {
      background: '#16a085',
      color: 'white'
    },
    buttonDisconnect: {
      background: '#c0392b',
      color: 'white'
    },
    refreshButton: {
      padding: '10px',
      borderRadius: '5px',
      border: '1px solid #4a9eff',
      background: '#0f3460',
      color: '#4a9eff',
      fontSize: '13px',
      cursor: 'pointer',
      fontWeight: 'bold'
    },
    error: {
      background: '#c0392b',
      padding: '10px',
      borderRadius: '5px',
      marginTop: '10px'
    },
    stats: {
      display: 'grid',
      gridTemplateColumns: 'repeat(3, 1fr)',
      gap: '10px',
      marginTop: '15px'
    },
    stat: {
      background: '#0f3460',
      padding: '10px',
      borderRadius: '5px',
      textAlign: 'center'
    },
    statLabel: {
      fontSize: '11px',
      color: '#888'
    },
    statValue: {
      fontSize: '20px',
      fontWeight: 'bold',
      color: '#4a9eff'
    },
    content: {
      display: 'flex',
      flex: 1,
      gap: '10px',
      padding: '10px',
      overflow: 'hidden'
    },
    viewer3d: {
      flex: 2,
      position: 'relative',
      background: '#0a0a0a',
      borderRadius: '8px',
      overflow: 'hidden'
    },
    imagePanel: {
      flex: 1,
      background: '#0f3460',
      borderRadius: '8px',
      padding: '15px',
      display: 'flex',
      flexDirection: 'column'
    },
    imageTitle: {
      marginBottom: '10px',
      fontSize: '16px',
      color: '#4a9eff'
    },
    imageContainer: {
      flex: 1,
      background: '#000',
      borderRadius: '5px',
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center',
      overflow: 'hidden'
    },
    image: {
      maxWidth: '100%',
      maxHeight: '100%',
      objectFit: 'contain'
    }
  };

  // Inicjalizacja Three.js
  useEffect(() => {
    if (!mountRef.current) return;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0a0a);
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
      requestAnimationFrame(animate);
      
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
      if (mountRef.current && renderer.domElement) {
        mountRef.current.removeChild(renderer.domElement);
      }
      renderer.dispose();
    };
  }, []);

  // Pobierz listę topiców z ROS
  const getTopics = () => {
    if (!rosRef.current?.ros) {
      setError('Najpierw połącz się z ROS');
      return;
    }

    setLoadingTopics(true);
    
    rosRef.current.ros.getTopics((result) => {
      const topics = result.topics || [];
      const types = result.types || [];
      
      const topicList = topics.map((topic, index) => ({
        name: topic,
        type: types[index]
      }));
      
      setAvailableTopics(topicList);
      setLoadingTopics(false);
      console.log('📋 Dostępne topiki:', topicList);
    }, (error) => {
      console.error('Błąd pobierania topiców:', error);
      setError('Nie można pobrać listy topiców');
      setLoadingTopics(false);
    });
  };

  // Zmień subskrypcję topiku chmury punktów
  const changePointCloudTopic = (newTopic) => {
    if (!rosRef.current?.ros) return;
    
    if (rosRef.current.pointCloudListener) {
      rosRef.current.pointCloudListener.unsubscribe();
    }
    
    const listener = new ROSLIB.Topic({
      ros: rosRef.current.ros,
      name: newTopic,
      messageType: 'sensor_msgs/PointCloud2'
    });
    listener.subscribe(processPointCloud);
    
    rosRef.current.pointCloudListener = listener;
    setSelectedPointCloudTopic(newTopic);
    console.log('📊 Zmieniono topic chmury punktów na:', newTopic);
  };

  // Zmień subskrypcję topiku obrazu
  const changeImageTopic = (newTopic) => {
    if (!rosRef.current?.ros) return;
    
    if (rosRef.current.imageListener) {
      rosRef.current.imageListener.unsubscribe();
    }
    
    const listener = new ROSLIB.Topic({
      ros: rosRef.current.ros,
      name: newTopic,
      messageType: 'sensor_msgs/Image'
    });
    listener.subscribe(processImage);
    
    rosRef.current.imageListener = listener;
    setSelectedImageTopic(newTopic);
    console.log('📷 Zmieniono topic obrazu na:', newTopic);
  };

  // Przetwarzanie PointCloud2
const processPointCloud = (message) => {
  try {
    const points = [];
    const colors = [];
    
    const data = new Uint8Array(message.data);
    const pointStep = message.point_step;
    const numPoints = message.width * message.height;
    
    // Znajdź offsety z fields
    const fields = message.fields;
    const xOffset = fields.find(f => f.name === 'x')?.offset || 0;
    const yOffset = fields.find(f => f.name === 'y')?.offset || 4;
    const zOffset = fields.find(f => f.name === 'z')?.offset || 8;
    
    let validPoints = 0;
    
    // NAPRAW: DataView musi mieć cały buffer
    const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
    
    for (let i = 0; i < numPoints; i++) {
      const offset = i * pointStep;
      
      // ZABEZPIECZENIE
      if (offset + zOffset + 4 > data.length) break;
      
      const x = view.getFloat32(offset + xOffset, true);
      const y = view.getFloat32(offset + yOffset, true);
      const z = view.getFloat32(offset + zOffset, true);
      
      if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
      
      points.push(x, y, z);
      validPoints++;
      
      const normalizedZ = Math.max(0, Math.min(1, (z + 2) / 4));
      colors.push(1 - normalizedZ, normalizedZ * 0.5, normalizedZ);
    }
    
    if (pointCloudRef.current && points.length > 0) {
      pointCloudRef.current.geometry.setAttribute(
        'position',
        new THREE.BufferAttribute(new Float32Array(points), 3)
      );
      pointCloudRef.current.geometry.setAttribute(
        'color',
        new THREE.BufferAttribute(new Float32Array(colors), 3)
      );
      
      pointCloudRef.current.geometry.computeBoundingSphere();
      pointCloudRef.current.geometry.attributes.position.needsUpdate = true;
      pointCloudRef.current.geometry.attributes.color.needsUpdate = true;
    }
    
    setPointCount(validPoints);
  } catch (err) {
    console.error('Błąd przetwarzania chmury punktów:', err);
    console.log('Message:', message); // DEBUG
  }
};

  // Przetwarzanie Image
  const processImage = (message) => {
    try {
      const width = message.width;
      const height = message.height;
      const encoding = message.encoding;
      
      if (!canvasRef.current) return;
      
      const canvas = canvasRef.current;
      canvas.width = width;
      canvas.height = height;
      const ctx = canvas.getContext('2d');
      
      const imageData = ctx.createImageData(width, height);
      const data = new Uint8Array(message.data);
      
      if (encoding === 'rgb8') {
        for (let i = 0; i < width * height; i++) {
          imageData.data[i * 4] = data[i * 3];
          imageData.data[i * 4 + 1] = data[i * 3 + 1];
          imageData.data[i * 4 + 2] = data[i * 3 + 2];
          imageData.data[i * 4 + 3] = 255;
        }
      } else if (encoding === 'bgr8') {
        for (let i = 0; i < width * height; i++) {
          imageData.data[i * 4] = data[i * 3 + 2];
          imageData.data[i * 4 + 1] = data[i * 3 + 1];
          imageData.data[i * 4 + 2] = data[i * 3];
          imageData.data[i * 4 + 3] = 255;
        }
      } else if (encoding === 'mono8' || encoding === 'mono16') {
        for (let i = 0; i < width * height; i++) {
          const gray = data[i];
          imageData.data[i * 4] = gray;
          imageData.data[i * 4 + 1] = gray;
          imageData.data[i * 4 + 2] = gray;
          imageData.data[i * 4 + 3] = 255;
        }
      }
      
      ctx.putImageData(imageData, 0, 0);
      setImageData(canvas.toDataURL());
    } catch (err) {
      console.error('Błąd przetwarzania obrazu:', err);
    }
  };

  // Połączenie z ROS
  const connectToROS = () => {
    setError('');
    
    try {
      const ros = new ROSLIB.Ros({ url: rosUrl });

      ros.on('connection', () => {
        console.log('✅ Połączono z ROS2');
        setConnected(true);
        setError('');
        
        // Automatycznie pobierz listę topiców
        setTimeout(() => {
          rosRef.current = { ros };
          getTopics();
        }, 500);
        
        // Subskrypcja chmury punktów
        const pointCloudListener = new ROSLIB.Topic({
          ros: ros,
          name: selectedPointCloudTopic,
          messageType: 'sensor_msgs/PointCloud2'
        });
        pointCloudListener.subscribe(processPointCloud);
        
        // Subskrypcja obrazu
        const imageListener = new ROSLIB.Topic({
          ros: ros,
          name: selectedImageTopic,
          messageType: 'sensor_msgs/Image'
        });
        imageListener.subscribe(processImage);

        rosRef.current = { ros, pointCloudListener, imageListener };
      });

      ros.on('error', (error) => {
        console.error('❌ Błąd ROS:', error);
        setError('Błąd połączenia: ' + error);
        setConnected(false);
      });

      ros.on('close', () => {
        console.log('🔌 Rozłączono');
        disconnect();
      });

    } catch (err) {
      setError('Błąd: ' + err.message);
      setConnected(false);
    }
  };

  const disconnect = () => {
    if (rosRef.current?.pointCloudListener) {
      rosRef.current.pointCloudListener.unsubscribe();
    }
    if (rosRef.current?.imageListener) {
      rosRef.current.imageListener.unsubscribe();
    }
    if (rosRef.current?.ros) {
      rosRef.current.ros.close();
    }
    setConnected(false);
    setPointCount(0);
    setImageData(null);
    setAvailableTopics([]);
  };

  return (
    <div style={styles.container}>
      {/* Header */}
      <div style={styles.header}>
        <h1 style={styles.title}>🎯 ROS2 Viewer - Point Cloud + Image</h1>
        
        <div style={styles.controls}>
          <div>
            <label>ROS Bridge URL</label>
            <input
              type="text"
              value={rosUrl}
              onChange={(e) => setRosUrl(e.target.value)}
              disabled={connected}
              style={styles.input}
            />
          </div>
          
          <div>
            <button
              onClick={connected ? disconnect : connectToROS}
              style={{
                ...styles.button,
                ...(connected ? styles.buttonDisconnect : styles.buttonConnect)
              }}
            >
              {connected ? '🔴 Rozłącz' : '🟢 Połącz'}
            </button>
          </div>
        </div>

        {error && <div style={styles.error}>⚠️ {error}</div>}

        {/* Wybór topiców */}
        {connected && (
          <div>
            <div style={styles.topicControls}>
              <div>
                <label>📊 Point Cloud Topic</label>
                <select
                  value={selectedPointCloudTopic}
                  onChange={(e) => changePointCloudTopic(e.target.value)}
                  style={styles.select}
                >
                  {availableTopics
                    .filter(t => t.type === 'sensor_msgs/PointCloud2' || t.type === 'sensor_msgs/msg/PointCloud2')
                    .map(topic => (
                      <option key={topic.name} value={topic.name}>
                        {topic.name}
                      </option>
                    ))
                  }
                  {availableTopics.filter(t => t.type === 'sensor_msgs/PointCloud2' || t.type === 'sensor_msgs/msg/PointCloud2').length === 0 && (
                    <option value={selectedPointCloudTopic}>{selectedPointCloudTopic}</option>
                  )}
                </select>
              </div>

              <div>
                <label>📷 Image Topic</label>
                <select
                  value={selectedImageTopic}
                  onChange={(e) => changeImageTopic(e.target.value)}
                  style={styles.select}
                >
                  {availableTopics
                    .filter(t => t.type === 'sensor_msgs/Image' || t.type === 'sensor_msgs/msg/Image')
                    .map(topic => (
                      <option key={topic.name} value={topic.name}>
                        {topic.name}
                      </option>
                    ))
                  }
                  {availableTopics.filter(t => t.type === 'sensor_msgs/Image' || t.type === 'sensor_msgs/msg/Image').length === 0 && (
                    <option value={selectedImageTopic}>{selectedImageTopic}</option>
                  )}
                </select>
              </div>
            </div>

            <button 
              onClick={getTopics} 
              style={styles.refreshButton}
              disabled={loadingTopics}
            >
              {loadingTopics ? '⏳ Ładowanie...' : '🔄 Odśwież listę topiców'}
            </button>
          </div>
        )}

        {connected && (
          <div style={styles.stats}>
            <div style={styles.stat}>
              <div style={styles.statLabel}>Status</div>
              <div style={styles.statValue}>Połączono</div>
            </div>
            <div style={styles.stat}>
              <div style={styles.statLabel}>Punkty</div>
              <div style={styles.statValue}>{pointCount}</div>
            </div>
            <div style={styles.stat}>
              <div style={styles.statLabel}>Obraz</div>
              <div style={styles.statValue}>{imageData ? '✓' : '✗'}</div>
            </div>
          </div>
        )}
      </div>

      {/* Główna zawartość */}
      <div style={styles.content}>
        {/* Panel 3D */}
        <div style={styles.viewer3d} ref={mountRef}></div>

        {/* Panel obrazu */}
        <div style={styles.imagePanel}>
          <div style={styles.imageTitle}>📷 Camera Image</div>
          <div style={styles.imageContainer}>
            {imageData ? (
              <img src={imageData} alt="Camera feed" style={styles.image} />
            ) : (
              <div style={{ color: '#666' }}>Brak obrazu</div>
            )}
          </div>
          <canvas ref={canvasRef} style={{ display: 'none' }} />
        </div>
      </div>
    </div>
  );
};

export default RosViewer;
