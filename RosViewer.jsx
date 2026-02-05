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
  
  // Konfiguracja widoku
  const [pointSize, setPointSize] = useState(0.05);
  const [colorMode, setColorMode] = useState('height');
  const [accumulatePoints, setAccumulatePoints] = useState(false); // MAPOWANIE!
  const [maxPoints, setMaxPoints] = useState(50000); // Limit punktów w mapie
  
  // Obroty sceny (nowe!)
  const [rotationX, setRotationX] = useState(0);
  const [rotationY, setRotationY] = useState(0);
  const [rotationZ, setRotationZ] = useState(0);
  
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
  
  // Bufor punktów dla mapowania
  const accumulatedPointsRef = useRef([]);
  const accumulatedColorsRef = useRef([]);

  // --- STYLE ---
  const styles = {
    container: { display: 'flex', flexDirection: 'column', height: '100vh', background: '#1a1a2e', color: 'white', fontFamily: 'Segoe UI, sans-serif' },
    header: { background: '#16213e', padding: '15px', borderBottom: '1px solid #2a2a4e', display: 'flex', flexWrap: 'wrap', gap: '20px', alignItems: 'center' },
    panelTitle: { color: '#4a9eff', margin: 0, fontSize: '18px', fontWeight: '600' },
    controlGroup: { display: 'flex', gap: '10px', alignItems: 'center' },
    input: { padding: '8px', borderRadius: '4px', border: '1px solid #444', background: '#0f3460', color: 'white' },
    button: { padding: '8px 16px', borderRadius: '4px', border: 'none', fontWeight: 'bold', cursor: 'pointer', color: 'white' },
    btnConnect: { background: '#16a085' },
    btnDisconnect: { background: '#c0392b' },
    btnSmall: { padding: '4px 8px', fontSize: '11px', background: '#2a4a7c' },
    content: { display: 'flex', flex: 1, overflow: 'hidden' },
    sidebar: { width: '300px', background: '#111', padding: '15px', display: 'flex', flexDirection: 'column', gap: '20px', overflowY: 'auto' },
    mainView: { flex: 1, position: 'relative', background: '#000' },
    settingRow: { display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '8px' },
    label: { fontSize: '13px', color: '#aaa' },
    select: { width: '100%', padding: '6px', background: '#222', color: '#fff', border: '1px solid #444', borderRadius: '4px' },
    imagePreview: { width: '100%', aspectRatio: '16/9', background: '#000', border: '1px solid #333', marginTop: '5px', objectFit: 'contain' },
    overlayStats: { position: 'absolute', top: 10, left: 10, background: 'rgba(0,0,0,0.8)', padding: '10px', borderRadius: '4px', pointerEvents: 'none', fontSize: '12px', fontFamily: 'monospace' },
    rotationControls: { display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '5px', marginTop: '10px' }
  };

  // --- INICJALIZACJA THREE.JS ---
  useEffect(() => {
    if (!mountRef.current) return;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x111111);
    
    const gridHelper = new THREE.GridHelper(20, 20, 0x444444, 0x222222);
    scene.add(gridHelper);
    
    const axesHelper = new THREE.AxesHelper(2);
    scene.add(axesHelper);

    const width = mountRef.current.clientWidth;
    const height = mountRef.current.clientHeight;
    const camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
    camera.position.set(5, 5, 8);

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    mountRef.current.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
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

  // --- DEKODOWANIE BASE64 ---
  const decodeBase64 = (base64String) => {
    const binaryString = window.atob(base64String);
    const len = binaryString.length;
    const bytes = new Uint8Array(len);
    for (let i = 0; i < len; i++) bytes[i] = binaryString.charCodeAt(i);
    return bytes;
  };

  // --- PRZETWARZANIE CHMURY Z MAPOWANIEM ---
  const processPointCloud = (message) => {
    try {
      const data = decodeBase64(message.data);
      const view = new DataView(data.buffer);
      
      const fields = message.fields;
      const xOff = fields.find(f => f.name === 'x')?.offset ?? 0;
      const yOff = fields.find(f => f.name === 'y')?.offset ?? 4;
      const zOff = fields.find(f => f.name === 'z')?.offset ?? 8;
      const pointStep = message.point_step;
      const numPoints = message.width * message.height;

      const newPositions = [];
      const newColors = [];

      const minZ = -2;
      const maxZ = 3;
      const rangeZ = maxZ - minZ;

      for (let i = 0; i < numPoints; i++) {
        const offset = i * pointStep;
        if (offset + 12 > data.length) break;

        const x = view.getFloat32(offset + xOff, true);
        const y = view.getFloat32(offset + yOff, true);
        const z = view.getFloat32(offset + zOff, true);

        if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
        if (x === 0 && y === 0 && z === 0) continue;

        newPositions.push(x, y, z);

        if (colorMode === 'solid') {
          newColors.push(1, 1, 1);
        } else {
          let norm = (z - minZ) / rangeZ;
          norm = Math.max(0, Math.min(1, norm));
          newColors.push(norm, 1 - Math.abs(0.5 - norm) * 2, 1 - norm);
        }
      }

      // MAPOWANIE - akumulacja punktów
      if (accumulatePoints) {
        // Dodaj nowe punkty do bufora
        accumulatedPointsRef.current.push(...newPositions);
        accumulatedColorsRef.current.push(...newColors);
        
        // Ogranicz liczbę punktów (FIFO - usuwaj najstarsze)
        const maxPointsCount = maxPoints * 3;
        if (accumulatedPointsRef.current.length > maxPointsCount) {
          const excess = accumulatedPointsRef.current.length - maxPointsCount;
          accumulatedPointsRef.current.splice(0, excess);
          accumulatedColorsRef.current.splice(0, excess);
        }
        
        // Renderuj całą mapę
        if (pointCloudRef.current && accumulatedPointsRef.current.length > 0) {
          const geometry = pointCloudRef.current.geometry;
          geometry.setAttribute('position', new THREE.Float32BufferAttribute(accumulatedPointsRef.current, 3));
          geometry.setAttribute('color', new THREE.Float32BufferAttribute(accumulatedColorsRef.current, 3));
          geometry.computeBoundingSphere();
          geometry.attributes.position.needsUpdate = true;
          geometry.attributes.color.needsUpdate = true;
        }
        
        setPointCount(newPositions.length / 3);
        setTotalPoints(accumulatedPointsRef.current.length / 3);
      } else {
        // Tryb normalny - tylko aktualna ramka
        if (pointCloudRef.current && newPositions.length > 0) {
          const geometry = pointCloudRef.current.geometry;
          geometry.setAttribute('position', new THREE.Float32BufferAttribute(newPositions, 3));
          geometry.setAttribute('color', new THREE.Float32BufferAttribute(newColors, 3));
          geometry.computeBoundingSphere();
          geometry.attributes.position.needsUpdate = true;
          geometry.attributes.color.needsUpdate = true;
        }
        setPointCount(newPositions.length / 3);
        setTotalPoints(newPositions.length / 3);
      }
    } catch (err) {
      console.error('Błąd przetwarzania chmury:', err);
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
      ros.on('error', e => setError('Błąd połączenia'));
      ros.on('close', () => setConnected(false));
    } catch (e) { 
      setError(e.message); 
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
            {connected ? 'Rozłącz' : 'Połącz'}
          </button>
        </div>
        {error && <span style={{color: '#e74c3c'}}>⚠️ {error}</span>}
      </div>

      {/* GŁÓWNA ZAWARTOŚĆ */}
      <div style={styles.content}>
        
        {/* LEWY PANEL */}
        <div style={styles.sidebar}>
          
          {/* KAMERA */}
          <div>
            <div style={styles.label}>KAMERA</div>
            <select style={styles.select} value={selectedImageTopic} onChange={e=>setSelectedImageTopic(e.target.value)}>
              {availableTopics.filter(t=>t.type.includes('Image')).map(t=><option key={t.name}>{t.name}</option>)}
            </select>
            <div style={styles.imagePreview}>
              {imageData ? <img src={imageData} style={{width:'100%', height:'100%', objectFit:'contain'}} alt="Camera" /> : <div style={{textAlign:'center', paddingTop:'20%', color:'#444'}}>Brak sygnału</div>}
            </div>
            <canvas ref={canvasRef} style={{display:'none'}} />
          </div>

          <hr style={{borderColor:'#333', width:'100%'}} />

          {/* CHMURA PUNKTÓW */}
          <div>
            <div style={styles.label}>CHMURA PUNKTÓW</div>
            <select style={{marginBottom:'15px', ...styles.select}} value={selectedPointCloudTopic} onChange={e=>setSelectedPointCloudTopic(e.target.value)}>
              {availableTopics.filter(t=>t.type.includes('Point')).map(t=><option key={t.name}>{t.name}</option>)}
            </select>

            {/* Rozmiar punktów */}
            <div style={styles.settingRow}>
              <span style={styles.label}>Rozmiar: {pointSize.toFixed(2)}</span>
            </div>
            <input type="range" min="0.01" max="0.5" step="0.01" value={pointSize} onChange={e => setPointSize(parseFloat(e.target.value))} style={{width: '100%'}} />

            {/* Kolorowanie */}
            <div style={{...styles.settingRow, marginTop:'10px'}}>
              <span style={styles.label}>Kolory</span>
              <select style={{...styles.select, width:'auto'}} value={colorMode} onChange={e => setColorMode(e.target.value)}>
                <option value="height">Wysokość</option>
                <option value="solid">Biały</option>
              </select>
            </div>

            {/* MAPOWANIE */}
            <div style={{...styles.settingRow, marginTop:'15px', background:'#1a3a5a', padding:'10px', borderRadius:'4px'}}>
              <div>
                <div style={{...styles.label, color:'#4a9eff', fontWeight:'bold'}}>🗺️ MAPOWANIE</div>
                <div style={{fontSize:'10px', color:'#888', marginTop:'3px'}}>Buduj mapę z ruchu robota</div>
              </div>
              <input type="checkbox" checked={accumulatePoints} onChange={e => {
                setAccumulatePoints(e.target.checked);
                if (!e.target.checked) clearMap();
              }} />
            </div>

            {accumulatePoints && (
              <div>
                <div style={styles.settingRow}>
                  <span style={styles.label}>Max punktów: {maxPoints}</span>
                </div>
                <input type="range" min="10000" max="200000" step="10000" value={maxPoints} onChange={e => setMaxPoints(parseInt(e.target.value))} style={{width: '100%'}} />
                <button onClick={clearMap} style={{...styles.button, ...styles.btnSmall, width:'100%', marginTop:'10px', background:'#c0392b'}}>
                  🗑️ Wyczyść mapę
                </button>
              </div>
            )}
          </div>

          <hr style={{borderColor:'#333', width:'100%'}} />

          {/* OBROTY SCENY */}
          <div>
            <div style={styles.label}>OBRÓT SCENY (°)</div>
            
            <div style={styles.settingRow}>
              <span style={styles.label}>Oś X: {rotationX}°</span>
            </div>
            <input type="range" min="-180" max="180" step="90" value={rotationX} onChange={e => setRotationX(parseInt(e.target.value))} style={{width: '100%'}} />
            
            <div style={styles.settingRow}>
              <span style={styles.label}>Oś Y: {rotationY}°</span>
            </div>
            <input type="range" min="-180" max="180" step="90" value={rotationY} onChange={e => setRotationY(parseInt(e.target.value))} style={{width: '100%'}} />
            
            <div style={styles.settingRow}>
              <span style={styles.label}>Oś Z: {rotationZ}°</span>
            </div>
            <input type="range" min="-180" max="180" step="90" value={rotationZ} onChange={e => setRotationZ(parseInt(e.target.value))} style={{width: '100%'}} />

            <div style={styles.rotationControls}>
              <button onClick={()=>{setRotationX(-90);setRotationY(0);setRotationZ(0)}} style={{...styles.button, ...styles.btnSmall}}>Z-up</button>
              <button onClick={()=>{setRotationX(0);setRotationY(0);setRotationZ(0)}} style={{...styles.button, ...styles.btnSmall}}>Reset</button>
              <button onClick={()=>{setRotationX(90);setRotationY(0);setRotationZ(180)}} style={{...styles.button, ...styles.btnSmall}}>Flip</button>
            </div>
          </div>
          
          <div style={{marginTop:'auto', fontSize:'11px', color:'#555', borderTop:'1px solid #333', paddingTop:'10px'}}>
            <b>Sterowanie:</b><br/>
            LPM = Obrót | PPM = Przesuw | Rolka = Zoom
          </div>
        </div>

        {/* WIDOK 3D */}
        <div style={styles.mainView} ref={mountRef}>
          <div style={styles.overlayStats}>
            Bieżąca ramka: {pointCount.toLocaleString()} pkt<br/>
            {accumulatePoints && <>Mapa: {totalPoints.toLocaleString()} pkt<br/></>}
            FPS: 60
          </div>
        </div>

      </div>
    </div>
  );
};

export default RosViewer;
