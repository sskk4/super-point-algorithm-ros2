import React, { useState, useEffect, useRef } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import * as ROSLIB from 'roslib';

const RosViewer = () => {
  // --- STANY ---
  const [connected, setConnected] = useState(false);
  const [rosUrl, setRosUrl] = useState('ws://192.168.1.100:9090');
  const [error, setError] = useState('');
  
  // Dane
  const [pointCount, setPointCount] = useState(0);
  const [totalPoints, setTotalPoints] = useState(0);
  const [imageData, setImageData] = useState(null);
  const [availableTopics, setAvailableTopics] = useState([]);
  const [fps, setFps] = useState(0);
  
  // Konfiguracja widoku
  const [pointSize, setPointSize] = useState(0.05);
  const [colorMode, setColorMode] = useState('height');
  const [accumulatePoints, setAccumulatePoints] = useState(false);
  const [maxPoints, setMaxPoints] = useState(100000);
  
  // Obroty sceny
  const [rotationX, setRotationX] = useState(-90);
  const [rotationY, setRotationY] = useState(0);
  const [rotationZ, setRotationZ] = useState(180);
  
  // DEBUG - Nowe parametry!
  const [updateRate, setUpdateRate] = useState(10);
  const [downsample, setDownsample] = useState(1);
  const [minDistance, setMinDistance] = useState(0);
  const [maxDistance, setMaxDistance] = useState(50);
  const [showStats, setShowStats] = useState(true);
  const [debugMode, setDebugMode] = useState(false);
  
  // ROI Filters - optymalizacja!
  const [minZ, setMinZ] = useState(-3);
  const [maxZ, setMaxZ] = useState(5);
  const [voxelSize, setVoxelSize] = useState(0); // 0 = wyłączone, 0.05 = 5cm woksele
  
  // TF Transform
  const [offsetX, setOffsetX] = useState(0);
  const [offsetY, setOffsetY] = useState(0);
  const [offsetZ, setOffsetZ] = useState(0);
  const [showOrigin, setShowOrigin] = useState(true);
  const [intensityFilter, setIntensityFilter] = useState(0);
  
  // Wybór topiców
  const [selectedPointCloudTopic, setSelectedPointCloudTopic] = useState('/point_cloud_1');
  const [selectedImageTopic, setSelectedImageTopic] = useState('/camera/image_raw');
  
  // Refs
  const mountRef = useRef(null);
  const canvasRef = useRef(null);
  const pointCloudRef = useRef(null);
  const rosRef = useRef(null);
  const rendererRef = useRef(null);
  const controlsRef = useRef(null);
  
  // Bufor punktów
  const accumulatedPointsRef = useRef([]);
  const accumulatedColorsRef = useRef([]);
  const originMarkerRef = useRef(null); // Marker pozycji LIDARa
  
  // Throttling dla update rate
  const lastUpdateTimeRef = useRef(0);
  const frameCountRef = useRef(0);
  const lastFpsTimeRef = useRef(Date.now());

  // --- STYLE ---
  const styles = {
    container: { display: 'flex', flexDirection: 'column', height: '100vh', background: '#1a1a2e', color: 'white', fontFamily: 'Segoe UI, sans-serif' },
    header: { background: '#16213e', padding: '15px', borderBottom: '1px solid #2a2a4e', display: 'flex', flexWrap: 'wrap', gap: '20px', alignItems: 'center' },
    panelTitle: { color: '#4a9eff', margin: 0, fontSize: '18px', fontWeight: '600' },
    controlGroup: { display: 'flex', gap: '10px', alignItems: 'center' },
    input: { padding: '8px', borderRadius: '4px', border: '1px solid #444', background: '#0f3460', color: 'white' },
    button: { padding: '8px 16px', borderRadius: '4px', border: 'none', fontWeight: 'bold', cursor: 'pointer', color: 'white', transition: 'all 0.2s' },
    btnConnect: { background: '#16a085' },
    btnDisconnect: { background: '#c0392b' },
    btnSmall: { padding: '4px 8px', fontSize: '11px', background: '#2a4a7c' },
    btnDebug: { padding: '6px 12px', fontSize: '12px', background: '#8e44ad' },
    content: { display: 'flex', flex: 1, overflow: 'hidden' },
    sidebar: { width: '320px', background: '#111', padding: '15px', display: 'flex', flexDirection: 'column', gap: '15px', overflowY: 'auto', fontSize: '13px' },
    mainView: { flex: 1, position: 'relative', background: '#000' },
    settingRow: { display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '5px' },
    label: { fontSize: '12px', color: '#aaa', fontWeight: '500' },
    select: { width: '100%', padding: '6px', background: '#222', color: '#fff', border: '1px solid #444', borderRadius: '4px' },
    imagePreview: { width: '100%', aspectRatio: '16/9', background: '#000', border: '1px solid #333', marginTop: '5px', objectFit: 'contain' },
    overlayStats: { position: 'absolute', top: 10, left: 10, background: 'rgba(0,0,0,0.85)', padding: '12px', borderRadius: '6px', pointerEvents: 'none', fontSize: '11px', fontFamily: 'Consolas, monospace', lineHeight: '1.6' },
    section: { background: '#0a0a1e', padding: '12px', borderRadius: '6px', marginBottom: '10px', border: '1px solid #2a2a4e' },
    sectionTitle: { color: '#4a9eff', fontSize: '13px', fontWeight: '600', marginBottom: '10px', display: 'flex', alignItems: 'center', gap: '8px' },
    debugSection: { background: '#1a0a2e', border: '1px solid #6a4a9e' },
    slider: { width: '100%', accentColor: '#4a9eff' },
    rotationControls: { display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '5px', marginTop: '8px' },
    valueDisplay: { fontSize: '11px', color: '#4a9eff', fontFamily: 'monospace' }
  };

  // --- INICJALIZACJA THREE.JS ---
  useEffect(() => {
    if (!mountRef.current) return;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0a0a);
    
    const gridHelper = new THREE.GridHelper(20, 20, 0x444444, 0x1a1a1a);
    scene.add(gridHelper);
    
    const axesHelper = new THREE.AxesHelper(3);
    scene.add(axesHelper);

    const width = mountRef.current.clientWidth;
    const height = mountRef.current.clientHeight;
    const camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
    camera.position.set(8, 8, 8);

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    mountRef.current.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.maxDistance = 100;
    controlsRef.current = controls;

    const geometry = new THREE.BufferGeometry();
    const material = new THREE.PointsMaterial({ 
      size: pointSize, 
      vertexColors: true,
      sizeAttenuation: true 
    });
    const points = new THREE.Points(geometry, material);
    scene.add(points);
    pointCloudRef.current = points;

    // Marker pozycji LIDARa (czerwona kula)
    const originGeometry = new THREE.SphereGeometry(0.1, 16, 16);
    const originMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
    const originMarker = new THREE.Mesh(originGeometry, originMaterial);
    originMarker.visible = showOrigin;
    scene.add(originMarker);
    originMarkerRef.current = originMarker;

    const handleResize = () => {
      if (!mountRef.current) return;
      const w = mountRef.current.clientWidth;
      const h = mountRef.current.clientHeight;
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
    };
    window.addEventListener('resize', handleResize);

    const animate = () => {
      requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
      
      // FPS counter
      frameCountRef.current++;
      const now = Date.now();
      if (now - lastFpsTimeRef.current >= 1000) {
        setFps(frameCountRef.current);
        frameCountRef.current = 0;
        lastFpsTimeRef.current = now;
      }
    };
    animate();

    return () => {
      window.removeEventListener('resize', handleResize);
      if (mountRef.current && renderer.domElement) {
        mountRef.current.removeChild(renderer.domElement);
      }
      renderer.dispose();
      controls.dispose();
    };
  }, []);

  // --- AKTUALIZACJA USTAWIEŃ ---
  useEffect(() => {
    if (pointCloudRef.current) {
      pointCloudRef.current.material.size = pointSize;
      pointCloudRef.current.material.needsUpdate = true;
    }
  }, [pointSize]);

  // Aktualizacja rotacji
  useEffect(() => {
    if (pointCloudRef.current) {
      pointCloudRef.current.rotation.x = rotationX * Math.PI / 180;
      pointCloudRef.current.rotation.y = rotationY * Math.PI / 180;
      pointCloudRef.current.rotation.z = rotationZ * Math.PI / 180;
    }
  }, [rotationX, rotationY, rotationZ]);

  // Aktualizacja pozycji origin marker i TF offset
  useEffect(() => {
    if (originMarkerRef.current) {
      originMarkerRef.current.visible = showOrigin;
      originMarkerRef.current.position.set(offsetX, offsetY, offsetZ);
    }
  }, [showOrigin, offsetX, offsetY, offsetZ]);

  // --- DEKODOWANIE BASE64 ---
  const decodeBase64 = (base64String) => {
    const binaryString = window.atob(base64String);
    const len = binaryString.length;
    const bytes = new Uint8Array(len);
    for (let i = 0; i < len; i++) bytes[i] = binaryString.charCodeAt(i);
    return bytes;
  };

  // --- PRZETWARZANIE CHMURY - ZOPTYMALIZOWANE ---
  const processPointCloud = (message) => {
    try {
      // THROTTLING
      const now = Date.now();
      const minInterval = 1000 / updateRate;
      if (now - lastUpdateTimeRef.current < minInterval) return;
      lastUpdateTimeRef.current = now;

      const data = decodeBase64(message.data);
      const view = new DataView(data.buffer);
      
      const fields = message.fields;
      const xOff = fields.find(f => f.name === 'x')?.offset ?? 0;
      const yOff = fields.find(f => f.name === 'y')?.offset ?? 4;
      const zOff = fields.find(f => f.name === 'z')?.offset ?? 8;
      
      const intensityField = fields.find(f => f.name === 'intensity' || f.name === 'i');
      const hasIntensity = intensityField !== undefined;
      const intensityOff = hasIntensity ? intensityField.offset : 0;
      
      const pointStep = message.point_step;
      const numPoints = message.width * message.height;

      // PRE-ALLOCATE TypedArrays (szybsze!)
      const maxPossiblePoints = Math.ceil(numPoints / downsample);
      const tempPositions = new Float32Array(maxPossiblePoints * 3);
      const tempColors = new Float32Array(maxPossiblePoints * 3);
      
      // KWADRATY odległości (bez Math.sqrt!)
      const minDistSq = minDistance * minDistance;
      const maxDistSq = maxDistance * maxDistance;

      // Voxel grid dla downsamplingu (jeśli włączony)
      const voxelMap = voxelSize > 0 ? new Map() : null;
      
      const rangeZ = maxZ - minZ;
      let validCount = 0;
      let filteredCount = 0;

      // GŁÓWNA PĘTLA - ZOPTYMALIZOWANA
      for (let i = 0; i < numPoints; i += downsample) {
        const offset = i * pointStep;
        if (offset + 12 > data.length) break;

        let x = view.getFloat32(offset + xOff, true);
        let y = view.getFloat32(offset + yOff, true);
        let z = view.getFloat32(offset + zOff, true);

        // 1. WALIDACJA (najszybsza)
        if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
        if (x === 0 && y === 0 && z === 0) continue;

        // 2. ROI Z-FILTER (przed obliczeniami!)
        if (z < minZ || z > maxZ) {
          filteredCount++;
          continue;
        }

        // 3. DISTANCE FILTER (kwadrat - BEZ sqrt!)
        const distSq = x*x + y*y + z*z;
        if (distSq < minDistSq || distSq > maxDistSq) {
          filteredCount++;
          continue;
        }

        // 4. INTENSYWNOŚĆ (jeśli dostępne)
        let intensity = 1;
        if (hasIntensity) {
          intensity = view.getFloat32(offset + intensityOff, true);
          if (intensityFilter > 0 && intensity < intensityFilter) {
            filteredCount++;
            continue;
          }
        }

        // 5. TF TRANSFORM
        x += offsetX;
        y += offsetY;
        z += offsetZ;

        // 6. VOXEL GRID DOWNSAMPLING (opcjonalnie)
        if (voxelMap) {
          const vx = Math.floor(x / voxelSize);
          const vy = Math.floor(y / voxelSize);
          const vz = Math.floor(z / voxelSize);
          const voxelKey = `${vx},${vy},${vz}`;
          
          if (voxelMap.has(voxelKey)) {
            filteredCount++;
            continue;
          }
          voxelMap.set(voxelKey, true);
        }

        // 7. DODAJ PUNKT
        const idx = validCount * 3;
        tempPositions[idx] = x;
        tempPositions[idx + 1] = y;
        tempPositions[idx + 2] = z;

        // 8. KOLOR
        if (colorMode === 'solid') {
          tempColors[idx] = tempColors[idx + 1] = tempColors[idx + 2] = 1;
        } else if (colorMode === 'distance') {
          // Użyj sqrt tylko dla koloru (nie do filtrowania!)
          const dist = Math.sqrt(distSq);
          const norm = Math.min(dist / maxDistance, 1);
          tempColors[idx] = 1 - norm;
          tempColors[idx + 1] = norm * 0.5;
          tempColors[idx + 2] = norm;
        } else if (colorMode === 'intensity' && hasIntensity) {
          const normInt = Math.min(intensity / 255, 1);
          tempColors[idx] = tempColors[idx + 1] = tempColors[idx + 2] = normInt;
        } else {
          // Wysokość (domyślny)
          let norm = (z - minZ) / rangeZ;
          norm = Math.max(0, Math.min(1, norm));
          tempColors[idx] = norm;
          tempColors[idx + 1] = 1 - Math.abs(0.5 - norm) * 2;
          tempColors[idx + 2] = 1 - norm;
        }

        validCount++;
      }

      // Debug
      if (debugMode) {
        console.log(`📊 Surowe: ${numPoints} | Przetworzone: ${validCount} | Odfiltrowane: ${filteredCount}`);
        if (hasIntensity) console.log('✅ Intensywność OK');
        if (voxelMap) console.log(`🔲 Voxel ${voxelSize}m: ${voxelMap.size} komórek`);
      }

      // Trim arrays do rzeczywistej wielkości
      const finalPositions = tempPositions.slice(0, validCount * 3);
      const finalColors = tempColors.slice(0, validCount * 3);

      // MAPOWANIE
      if (accumulatePoints) {
        // Konwertuj do zwykłych array dla akumulacji
        accumulatedPointsRef.current.push(...Array.from(finalPositions));
        accumulatedColorsRef.current.push(...Array.from(finalColors));
        
        const maxPointsCount = maxPoints * 3;
        if (accumulatedPointsRef.current.length > maxPointsCount) {
          const excess = accumulatedPointsRef.current.length - maxPointsCount;
          accumulatedPointsRef.current.splice(0, excess);
          accumulatedColorsRef.current.splice(0, excess);
        }
        
        if (pointCloudRef.current && accumulatedPointsRef.current.length > 0) {
          const geometry = pointCloudRef.current.geometry;
          geometry.setAttribute('position', new THREE.Float32BufferAttribute(accumulatedPointsRef.current, 3));
          geometry.setAttribute('color', new THREE.Float32BufferAttribute(accumulatedColorsRef.current, 3));
          geometry.computeBoundingSphere();
          geometry.attributes.position.needsUpdate = true;
          geometry.attributes.color.needsUpdate = true;
        }
        
        setPointCount(validCount);
        setTotalPoints(accumulatedPointsRef.current.length / 3);
      } else {
        // Tryb normalny - użyj TypedArray bezpośrednio
        if (pointCloudRef.current && validCount > 0) {
          const geometry = pointCloudRef.current.geometry;
          geometry.setAttribute('position', new THREE.Float32BufferAttribute(finalPositions, 3));
          geometry.setAttribute('color', new THREE.Float32BufferAttribute(finalColors, 3));
          geometry.computeBoundingSphere();
          geometry.attributes.position.needsUpdate = true;
          geometry.attributes.color.needsUpdate = true;
        }
        setPointCount(validCount);
        setTotalPoints(validCount);
      }
    } catch (err) {
      console.error('❌ Błąd:', err);
    }
  };

  // Wyczyść mapę
  const clearMap = () => {
    accumulatedPointsRef.current = [];
    accumulatedColorsRef.current = [];
    setTotalPoints(0);
    if (pointCloudRef.current) {
      const geometry = pointCloudRef.current.geometry;
      geometry.setAttribute('position', new THREE.Float32BufferAttribute([], 3));
      geometry.setAttribute('color', new THREE.Float32BufferAttribute([], 3));
    }
  };

  // --- PRZETWARZANIE OBRAZU ---
  const processImage = (message) => {
    try {
      const width = message.width;
      const height = message.height;
      const encoding = message.encoding;

      const canvas = canvasRef.current;
      if (!canvas) return;

      canvas.width = width;
      canvas.height = height;
      const ctx = canvas.getContext('2d');
      const imgData = ctx.createImageData(width, height);

      const data = decodeBase64(message.data);

      if (encoding === 'rgb8') {
        for (let i = 0; i < width * height; i++) {
          imgData.data[i * 4]     = data[i * 3];
          imgData.data[i * 4 + 1] = data[i * 3 + 1];
          imgData.data[i * 4 + 2] = data[i * 3 + 2];
          imgData.data[i * 4 + 3] = 255;
        }
      } else if (encoding === 'bgr8') {
        for (let i = 0; i < width * height; i++) {
          imgData.data[i * 4]     = data[i * 3 + 2];
          imgData.data[i * 4 + 1] = data[i * 3 + 1];
          imgData.data[i * 4 + 2] = data[i * 3];
          imgData.data[i * 4 + 3] = 255;
        }
      } else if (encoding.includes('mono')) {
        for (let i = 0; i < width * height; i++) {
          const g = data[i];
          imgData.data[i * 4] = imgData.data[i * 4 + 1] = imgData.data[i * 4 + 2] = g;
          imgData.data[i * 4 + 3] = 255;
        }
      }

      ctx.putImageData(imgData, 0, 0);
      setImageData(canvas.toDataURL());
    } catch (e) {
      console.error('Błąd obrazu:', e);
    }
  };

  // --- ROS CONNECTION ---
  const connect = () => {
    try {
      const ros = new ROSLIB.Ros({ url: rosUrl });
      ros.on('connection', () => {
        setConnected(true);
        setError('');
        rosRef.current = { ros };
        
        ros.getTopics((res) => {
          const list = res.topics.map((n, i) => ({ name: n, type: res.types[i] }));
          setAvailableTopics(list);
        });

        const pcSub = new ROSLIB.Topic({ ros, name: selectedPointCloudTopic, messageType: 'sensor_msgs/PointCloud2' });
        pcSub.subscribe(processPointCloud);
        
        const imgSub = new ROSLIB.Topic({ ros, name: selectedImageTopic, messageType: 'sensor_msgs/Image' });
        imgSub.subscribe(processImage);
        
        rosRef.current.pcSub = pcSub;
        rosRef.current.imgSub = imgSub;
      });
      ros.on('error', e => setError('Błąd połączenia: ' + e));
      ros.on('close', () => setConnected(false));
    } catch (e) { 
      setError('Błąd: ' + e.message); 
    }
  };
  
  const disconnect = () => {
    if(rosRef.current?.pcSub) rosRef.current.pcSub.unsubscribe();
    if(rosRef.current?.imgSub) rosRef.current.imgSub.unsubscribe();
    if(rosRef.current?.ros) rosRef.current.ros.close();
    setConnected(false);
  };

  return (
    <div style={styles.container}>
      {/* GÓRNY PASEK */}
      <div style={styles.header}>
        <h2 style={styles.panelTitle}>🚀 ROS2 Pro Viewer</h2>
        <div style={styles.controlGroup}>
          <input value={rosUrl} onChange={e=>setRosUrl(e.target.value)} style={styles.input} disabled={connected} />
          <button onClick={connected ? disconnect : connect} style={{...styles.button, ...(connected ? styles.btnDisconnect : styles.btnConnect)}}>
            {connected ? '🔴 Rozłącz' : '🟢 Połącz'}
          </button>
          <button onClick={()=>setDebugMode(!debugMode)} style={{...styles.button, ...styles.btnDebug}}>
            {debugMode ? '🐛 Debug ON' : '🐛 Debug'}
          </button>
        </div>
        {error && <span style={{color: '#e74c3c', fontSize:'12px'}}>⚠️ {error}</span>}
      </div>

      {/* GŁÓWNA ZAWARTOŚĆ */}
      <div style={styles.content}>
        
        {/* LEWY PANEL */}
        <div style={styles.sidebar}>
          
          {/* KAMERA */}
          <div style={styles.section}>
            <div style={styles.sectionTitle}>📷 KAMERA</div>
            <select style={styles.select} value={selectedImageTopic} onChange={e=>setSelectedImageTopic(e.target.value)}>
              {availableTopics.filter(t=>t.type.includes('Image')).map(t=><option key={t.name}>{t.name}</option>)}
            </select>
            <div style={styles.imagePreview}>
              {imageData ? <img src={imageData} style={{width:'100%', height:'100%', objectFit:'contain'}} alt="Camera" /> : <div style={{textAlign:'center', paddingTop:'20%', color:'#444', fontSize:'11px'}}>Brak sygnału</div>}
            </div>
            <canvas ref={canvasRef} style={{display:'none'}} />
          </div>

          {/* CHMURA PUNKTÓW */}
          <div style={styles.section}>
            <div style={styles.sectionTitle}>☁️ CHMURA PUNKTÓW</div>
            <select style={{marginBottom:'10px', ...styles.select}} value={selectedPointCloudTopic} onChange={e=>setSelectedPointCloudTopic(e.target.value)}>
              {availableTopics.filter(t=>t.type.includes('Point')).map(t=><option key={t.name}>{t.name}</option>)}
            </select>

            <div style={styles.settingRow}>
              <span style={styles.label}>Rozmiar punktów</span>
              <span style={styles.valueDisplay}>{pointSize.toFixed(2)}</span>
            </div>
            <input type="range" min="0.01" max="0.5" step="0.01" value={pointSize} onChange={e => setPointSize(parseFloat(e.target.value))} style={styles.slider} />

            <div style={styles.settingRow}>
              <span style={styles.label}>Kolorowanie</span>
              <select style={{...styles.select, width:'auto', fontSize:'11px'}} value={colorMode} onChange={e => setColorMode(e.target.value)}>
                <option value="height">Wysokość</option>
                <option value="distance">Odległość</option>
                <option value="intensity">Intensywność</option>
                <option value="solid">Jednolity</option>
              </select>
            </div>
          </div>

          {/* MAPOWANIE */}
          <div style={styles.section}>
            <div style={styles.sectionTitle}>
              🗺️ MAPOWANIE
              <input type="checkbox" checked={accumulatePoints} onChange={e => {
                setAccumulatePoints(e.target.checked);
                if (!e.target.checked) clearMap();
              }} style={{marginLeft:'auto'}} />
            </div>

            {accumulatePoints && (
              <>
                <div style={styles.settingRow}>
                  <span style={styles.label}>Max punktów</span>
                  <span style={styles.valueDisplay}>{(maxPoints/1000).toFixed(0)}k</span>
                </div>
                <input type="range" min="10000" max="500000" step="10000" value={maxPoints} onChange={e => setMaxPoints(parseInt(e.target.value))} style={styles.slider} />
                <button onClick={clearMap} style={{...styles.button, ...styles.btnSmall, width:'100%', marginTop:'8px', background:'#c0392b'}}>
                  🗑️ Wyczyść mapę
                </button>
              </>
            )}
          </div>

          {/* OBROTY */}
          <div style={styles.section}>
            <div style={styles.sectionTitle}>🔄 OBRÓT SCENY</div>
            
            <div style={styles.settingRow}>
              <span style={styles.label}>Oś X</span>
              <span style={styles.valueDisplay}>{rotationX}°</span>
            </div>
            <input type="range" min="-180" max="180" step="15" value={rotationX} onChange={e => setRotationX(parseInt(e.target.value))} style={styles.slider} />
            
            <div style={styles.settingRow}>
              <span style={styles.label}>Oś Y</span>
              <span style={styles.valueDisplay}>{rotationY}°</span>
            </div>
            <input type="range" min="-180" max="180" step="15" value={rotationY} onChange={e => setRotationY(parseInt(e.target.value))} style={styles.slider} />
            
            <div style={styles.settingRow}>
              <span style={styles.label}>Oś Z</span>
              <span style={styles.valueDisplay}>{rotationZ}°</span>
            </div>
            <input type="range" min="-180" max="180" step="15" value={rotationZ} onChange={e => setRotationZ(parseInt(e.target.value))} style={styles.slider} />

            <div style={styles.rotationControls}>
              <button onClick={()=>{setRotationX(-90);setRotationY(0);setRotationZ(180)}} style={{...styles.button, ...styles.btnSmall}}>Domyślny</button>
              <button onClick={()=>{setRotationX(0);setRotationY(0);setRotationZ(0)}} style={{...styles.button, ...styles.btnSmall}}>Reset</button>
              <button onClick={()=>{setRotationX(90);setRotationY(0);setRotationZ(0)}} style={{...styles.button, ...styles.btnSmall}}>Flip</button>
            </div>
          </div>

          {/* DEBUG */}
          {debugMode && (
            <div style={{...styles.section, ...styles.debugSection}}>
              <div style={styles.sectionTitle}>🐛 DEBUG & KALIBRACJA</div>
              
              <div style={styles.settingRow}>
                <span style={styles.label}>Update Rate</span>
                <span style={styles.valueDisplay}>{updateRate} Hz</span>
              </div>
              <input type="range" min="1" max="60" step="1" value={updateRate} onChange={e => setUpdateRate(parseInt(e.target.value))} style={styles.slider} />
              
              <div style={styles.settingRow}>
                <span style={styles.label}>Downsampling</span>
                <span style={styles.valueDisplay}>1/{downsample}</span>
              </div>
              <input type="range" min="1" max="10" step="1" value={downsample} onChange={e => setDownsample(parseInt(e.target.value))} style={styles.slider} />
              
              <div style={styles.settingRow}>
                <span style={styles.label}>Min odległość</span>
                <span style={styles.valueDisplay}>{minDistance.toFixed(1)}m</span>
              </div>
              <input type="range" min="0" max="5" step="0.1" value={minDistance} onChange={e => setMinDistance(parseFloat(e.target.value))} style={styles.slider} />
              
              <div style={styles.settingRow}>
                <span style={styles.label}>Max odległość</span>
                <span style={styles.valueDisplay}>{maxDistance}m</span>
              </div>
              <input type="range" min="1" max="100" step="1" value={maxDistance} onChange={e => setMaxDistance(parseInt(e.target.value))} style={styles.slider} />

              <hr style={{borderColor:'#444', margin:'12px 0'}} />

              <div style={{...styles.sectionTitle, fontSize:'12px', color:'#e67e22'}}>📍 POZYCJA LIDARA (TF Offset)</div>
              
              <div style={styles.settingRow}>
                <span style={styles.label}>Pokaż origin</span>
                <input type="checkbox" checked={showOrigin} onChange={e => setShowOrigin(e.target.checked)} />
              </div>

              <div style={styles.settingRow}>
                <span style={styles.label}>Offset X</span>
                <span style={styles.valueDisplay}>{offsetX.toFixed(2)}m</span>
              </div>
              <input type="range" min="-5" max="5" step="0.1" value={offsetX} onChange={e => setOffsetX(parseFloat(e.target.value))} style={styles.slider} />
              
              <div style={styles.settingRow}>
                <span style={styles.label}>Offset Y</span>
                <span style={styles.valueDisplay}>{offsetY.toFixed(2)}m</span>
              </div>
              <input type="range" min="-5" max="5" step="0.1" value={offsetY} onChange={e => setOffsetY(parseFloat(e.target.value))} style={styles.slider} />
              
              <div style={styles.settingRow}>
                <span style={styles.label}>Offset Z</span>
                <span style={styles.valueDisplay}>{offsetZ.toFixed(2)}m</span>
              </div>
              <input type="range" min="-5" max="5" step="0.1" value={offsetZ} onChange={e => setOffsetZ(parseFloat(e.target.value))} style={styles.slider} />

              <button onClick={()=>{setOffsetX(0);setOffsetY(0);setOffsetZ(0)}} style={{...styles.button, ...styles.btnSmall, width:'100%', marginTop:'8px'}}>
                ↺ Reset offsetów
              </button>

              <div style={{marginTop:'12px', fontSize:'10px', color:'#888', lineHeight:'1.5'}}>
                💡 <b>Update Rate</b> = ile razy/s aktualizować<br/>
                💡 <b>Downsampling</b> = co który punkt (mniej = szybciej)<br/>
                💡 <b>Filtr odległości</b> = usuń punkty za blisko/daleko<br/>
                💡 <b>TF Offset</b> = przesuń chmurę (czerwona kula = pozycja LIDARa)<br/>
                💡 Czerwona kula pokazuje gdzie jest LIDAR
              </div>

              <hr style={{borderColor:'#444', margin:'12px 0'}} />

              <div style={{...styles.sectionTitle, fontSize:'12px', color:'#3498db'}}>⚡ OPTYMALIZACJA</div>
              
              <div style={styles.settingRow}>
                <span style={styles.label}>Filtr Z min</span>
                <span style={styles.valueDisplay}>{minZ.toFixed(1)}m</span>
              </div>
              <input type="range" min="-10" max="0" step="0.5" value={minZ} onChange={e => setMinZ(parseFloat(e.target.value))} style={styles.slider} />
              
              <div style={styles.settingRow}>
                <span style={styles.label}>Filtr Z max</span>
                <span style={styles.valueDisplay}>{maxZ.toFixed(1)}m</span>
              </div>
              <input type="range" min="0" max="20" step="0.5" value={maxZ} onChange={e => setMaxZ(parseFloat(e.target.value))} style={styles.slider} />

              <div style={styles.settingRow}>
                <span style={styles.label}>Voxel grid</span>
                <span style={styles.valueDisplay}>{voxelSize === 0 ? 'OFF' : `${(voxelSize*100).toFixed(0)}cm`}</span>
              </div>
              <input type="range" min="0" max="0.5" step="0.05" value={voxelSize} onChange={e => setVoxelSize(parseFloat(e.target.value))} style={styles.slider} />

              <div style={{marginTop:'8px', fontSize:'10px', color:'#888', lineHeight:'1.5'}}>
                🚀 <b>Z-filter</b> = usuń punkty poza zakresem wysokości (szybkie!)<br/>
                🔲 <b>Voxel grid</b> = jeden punkt na komórkę (mniej punktów, szybciej)<br/>
                💡 Voxel 5cm = dobry kompromis jakość/prędkość
              </div>
            </div>
          )}
          
          <div style={{marginTop:'auto', fontSize:'10px', color:'#555', borderTop:'1px solid #222', paddingTop:'8px'}}>
            <b>Sterowanie:</b> LPM=Obrót | PPM=Przesuw | Scroll=Zoom
          </div>
        </div>

        {/* WIDOK 3D */}
        <div style={styles.mainView} ref={mountRef}>
          {showStats && (
            <div style={styles.overlayStats}>
              <div style={{color:'#4a9eff', fontWeight:'bold', marginBottom:'6px'}}>📊 STATYSTYKI</div>
              Ramka: <span style={{color:'#2ecc71'}}>{pointCount.toLocaleString()}</span> pkt<br/>
              {accumulatePoints && <>Mapa: <span style={{color:'#e67e22'}}>{totalPoints.toLocaleString()}</span> pkt<br/></>}
              Render: <span style={{color:'#3498db'}}>{fps}</span> FPS<br/>
              Update: <span style={{color:'#9b59b6'}}>{updateRate}</span> Hz<br/>
              {debugMode && <>
                Sample: <span style={{color:'#f39c12'}}>1/{downsample}</span><br/>
                Range: <span style={{color:'#e74c3c'}}>{minDistance.toFixed(1)}-{maxDistance}m</span>
              </>}
              <div style={{marginTop:'8px', fontSize:'10px', color:'#666'}}>
                {connected ? '🟢 Połączono' : '🔴 Rozłączono'}
              </div>
            </div>
          )}
        </div>

      </div>
    </div>
  );
};

export default RosViewer;
