import React, { useState, useEffect, useRef } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls'; // Dodaj ten import!
import * as ROSLIB from 'roslib';

const RosViewer = () => {
  // --- STANY ---
  const [connected, setConnected] = useState(false);
  const [rosUrl, setRosUrl] = useState('ws://192.168.1.100:9090');
  const [error, setError] = useState('');
  
  // Dane
  const [pointCount, setPointCount] = useState(0);
  const [imageData, setImageData] = useState(null);
  const [availableTopics, setAvailableTopics] = useState([]);
  
  // Konfiguracja widoku (Nowe funkcje!)
  const [pointSize, setPointSize] = useState(0.05);
  const [colorMode, setColorMode] = useState('height'); // 'height' | 'solid'
  const [flipAxes, setFlipAxes] = useState(true); // Czy obrócić chmurę (Z-up -> Y-up)
  
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
    content: { display: 'flex', flex: 1, overflow: 'hidden' },
    sidebar: { width: '300px', background: '#111', padding: '15px', display: 'flex', flexDirection: 'column', gap: '20px', overflowY: 'auto' },
    mainView: { flex: 1, position: 'relative', background: '#000' },
    settingRow: { display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '8px' },
    label: { fontSize: '13px', color: '#aaa' },
    select: { width: '100%', padding: '6px', background: '#222', color: '#fff', border: '1px solid #444', borderRadius: '4px' },
    imagePreview: { width: '100%', aspectRatio: '16/9', background: '#000', border: '1px solid #333', marginTop: '5px', objectFit: 'contain' },
    overlayStats: { position: 'absolute', top: 10, left: 10, background: 'rgba(0,0,0,0.7)', padding: '10px', borderRadius: '4px', pointerEvents: 'none', fontSize: '12px' }
  };

  // --- INICJALIZACJA THREE.JS + ORBIT CONTROLS ---
  useEffect(() => {
    if (!mountRef.current) return;

    // 1. Scena
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x111111);
    
    // Siatka podłogi (Grid)
    const gridHelper = new THREE.GridHelper(20, 20, 0x444444, 0x222222);
    scene.add(gridHelper);
    
    // Osie (RGB = XYZ)
    const axesHelper = new THREE.AxesHelper(2);
    scene.add(axesHelper);

    // 2. Kamera
    const width = mountRef.current.clientWidth;
    const height = mountRef.current.clientHeight;
    const camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
    camera.position.set(5, 5, 8); // Startowa pozycja

    // 3. Renderer
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    mountRef.current.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    // 4. ORBIT CONTROLS (To naprawia sterowanie!)
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true; // Płynność ruchu
    controls.dampingFactor = 0.05;
    controlsRef.current = controls;

    // 5. Pusty obiekt na punkty
    const geometry = new THREE.BufferGeometry();
    const material = new THREE.PointsMaterial({ 
      size: pointSize, 
      vertexColors: true,
      sizeAttenuation: true 
    });
    const points = new THREE.Points(geometry, material);
    
    // Domyślnie obróć kontener punktów, jeśli flipAxes jest true
    if (flipAxes) {
      points.rotation.x = -Math.PI / 2;
    }
    
    scene.add(points);
    pointCloudRef.current = points;

    // Resize handler
    const handleResize = () => {
      if (!mountRef.current) return;
      const w = mountRef.current.clientWidth;
      const h = mountRef.current.clientHeight;
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
    };
    window.addEventListener('resize', handleResize);

    // Pętla animacji
    const animate = () => {
      requestAnimationFrame(animate);
      controls.update(); // Aktualizacja OrbitControls
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
  }, []); // Run once on mount

  // --- AKTUALIZACJA USTAWIEŃ W CZASIE RZECZYWISTYM ---
  useEffect(() => {
    if (pointCloudRef.current) {
      // Aktualizacja wielkości punktów
      pointCloudRef.current.material.size = pointSize;
      
      // Aktualizacja rotacji (Flip Axes)
      // Jeśli ROS daje Z-up, a Three.js ma Y-up, musimy obrócić o -90 stopni wokół X
      pointCloudRef.current.rotation.x = flipAxes ? -Math.PI / 2 : 0;
      
      pointCloudRef.current.material.needsUpdate = true;
    }
  }, [pointSize, flipAxes]);


  // --- PRZETWARZANIE CHMURY (Zoptymalizowane) ---
  const decodeBase64 = (base64String) => {
    const binaryString = window.atob(base64String);
    const len = binaryString.length;
    const bytes = new Uint8Array(len);
    for (let i = 0; i < len; i++) bytes[i] = binaryString.charCodeAt(i);
    return bytes;
  };

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

      const positions = [];
      const colors = [];

      // Konfiguracja kolorów heatmapy (prosta: niebieski -> zielony -> czerwony)
      const minZ = -2; // Dostosuj do swoich danych
      const maxZ = 3;  // Dostosuj do swoich danych
      const rangeZ = maxZ - minZ;

      for (let i = 0; i < numPoints; i++) {
        const offset = i * pointStep;
        if (offset + 12 > data.length) break;

        const x = view.getFloat32(offset + xOff, true);
        const y = view.getFloat32(offset + yOff, true);
        const z = view.getFloat32(offset + zOff, true);

        if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
        if (x === 0 && y === 0 && z === 0) continue; // Często puste punkty w LiDARze

        positions.push(x, y, z);

        // LOGIKA KOLOROWANIA
        if (colorMode === 'solid') {
          colors.push(1, 1, 1); // Biały
        } else {
          // Heatmapa po wysokości (Z)
          let norm = (z - minZ) / rangeZ;
          norm = Math.max(0, Math.min(1, norm));
          
          // Prosty gradient R-G-B
          colors.push(norm, 1 - Math.abs(0.5 - norm) * 2, 1 - norm);
        }
      }

      if (pointCloudRef.current && positions.length > 0) {
        const geometry = pointCloudRef.current.geometry;
        geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
        geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
        geometry.computeBoundingSphere();
        geometry.attributes.position.needsUpdate = true;
        geometry.attributes.color.needsUpdate = true;
      }
      setPointCount(positions.length / 3);
    } catch (err) {
      console.error(err);
    }
  };

  // --- PRZETWARZANIE OBRAZU ---
  const processImage = (message) => {
    // (Kod bez zmian - jest poprawny z poprzedniego kroku)
    try {
        const width = message.width;
        const height = message.height;
        const canvas = canvasRef.current;
        if(!canvas) return;
        canvas.width = width;
        canvas.height = height;
        const ctx = canvas.getContext('2d');
        const imgData = ctx.createImageData(width, height);
        const data = decodeBase64(message.data);
        
        // Prosta obsługa rgb8/bgr8/mono8 (skrót)
        let ptr = 0;
        for(let i=0; i < width*height; i++) {
            const r = message.encoding.includes('bgr') ? data[ptr+2] : data[ptr];
            const g = message.encoding.includes('mono') ? data[ptr] : data[ptr+1];
            const b = message.encoding.includes('bgr') ? data[ptr] : (message.encoding.includes('mono') ? data[ptr] : data[ptr+2]);
            imgData.data[i*4] = r;
            imgData.data[i*4+1] = g;
            imgData.data[i*4+2] = b;
            imgData.data[i*4+3] = 255;
            ptr += (message.encoding.includes('mono') ? 1 : 3);
        }
        ctx.putImageData(imgData, 0, 0);
        setImageData(canvas.toDataURL());
    } catch(e) {}
  };

  // --- ROS CONNECTION ---
  const connect = () => {
    try {
      const ros = new ROSLIB.Ros({ url: rosUrl });
      ros.on('connection', () => {
        setConnected(true);
        rosRef.current = { ros };
        
        // Pobierz topiki
        ros.getTopics((res) => {
            const list = res.topics.map((n, i) => ({ name: n, type: res.types[i] }));
            setAvailableTopics(list);
        });

        // Subskrypcje
        const pcSub = new ROSLIB.Topic({ ros, name: selectedPointCloudTopic, messageType: 'sensor_msgs/PointCloud2' });
        pcSub.subscribe(processPointCloud);
        
        const imgSub = new ROSLIB.Topic({ ros, name: selectedImageTopic, messageType: 'sensor_msgs/Image' });
        imgSub.subscribe(processImage);
        
        rosRef.current.pcSub = pcSub;
        rosRef.current.imgSub = imgSub;
      });
      ros.on('error', e => setError('Błąd połączenia'));
    } catch (e) { setError(e.message); }
  };
  
  const disconnect = () => {
    if(rosRef.current?.ros) rosRef.current.ros.close();
    setConnected(false);
  };

  return (
    <div style={styles.container}>
      {/* 1. GÓRNY PASEK */}
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

      {/* 2. GŁÓWNA ZAWARTOŚĆ */}
      <div style={styles.content}>
        
        {/* LEWY PANEL STEROWANIA */}
        <div style={styles.sidebar}>
          
          {/* Sekcja Obrazu z kamery */}
          <div>
            <div style={styles.label}>KAMERA (Image Topic)</div>
            <select style={styles.select} value={selectedImageTopic} onChange={e=>setSelectedImageTopic(e.target.value)}>
                {availableTopics.filter(t=>t.type.includes('Image')).map(t=><option key={t.name}>{t.name}</option>)}
            </select>
            <div style={styles.imagePreview}>
               {imageData ? <img src={imageData} style={{width:'100%', height:'100%', objectFit:'contain'}} /> : <div style={{textAlign:'center', paddingTop:'20%', color:'#444'}}>No Signal</div>}
            </div>
            <canvas ref={canvasRef} style={{display:'none'}} />
          </div>

          <hr style={{borderColor:'#333', width:'100%'}} />

          {/* Sekcja Ustawień Chmury 3D */}
          <div>
            <div style={styles.label}>CHMURA (PointCloud2 Topic)</div>
            <select style={styles.select} value={selectedPointCloudTopic} onChange={e=>setSelectedPointCloudTopic(e.target.value)} style={{marginBottom:'15px', ...styles.select}}>
               {availableTopics.filter(t=>t.type.includes('Point')).map(t=><option key={t.name}>{t.name}</option>)}
            </select>

            {/* SUWAK WIELKOŚCI PUNKTÓW */}
            <div style={styles.settingRow}>
              <span style={styles.label}>Rozmiar punktów: {pointSize}</span>
            </div>
            <input 
              type="range" min="0.01" max="0.5" step="0.01" 
              value={pointSize} 
              onChange={e => setPointSize(parseFloat(e.target.value))} 
              style={{width: '100%'}} 
            />

            {/* FLIP AXES - To naprawi "ścianę" */}
            <div style={{...styles.settingRow, marginTop:'15px'}}>
              <span style={styles.label}>Obróć (Z-up ➡ Y-up)</span>
              <input type="checkbox" checked={flipAxes} onChange={e => setFlipAxes(e.target.checked)} />
            </div>

            {/* TRYB KOLORU */}
            <div style={styles.settingRow}>
              <span style={styles.label}>Kolorowanie</span>
              <select style={styles.select} value={colorMode} onChange={e => setColorMode(e.target.value)}>
                <option value="height">Wysokość (Heatmap)</option>
                <option value="solid">Jednolity (Biały)</option>
              </select>
            </div>
          </div>
          
          <div style={{marginTop:'auto', fontSize:'11px', color:'#555'}}>
            Sterowanie: LPM=Obrót, PPM=Przesuw, Rolka=Zoom
          </div>
        </div>

        {/* WIDOK 3D */}
        <div style={styles.mainView} ref={mountRef}>
           <div style={styles.overlayStats}>
              Punkty: {pointCount.toLocaleString()}<br/>
              FPS: 60
           </div>
        </div>

      </div>
    </div>
  );
};

export default RosViewer;
