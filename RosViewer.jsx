import React, { useState, useEffect, useRef } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import ROSLIB from 'roslib';

const RosViewer = () => {
  // --- STANY ---
  const [connected, setConnected] = useState(false);
  const [rosUrl, setRosUrl] = useState('ws://192.168.1.100:9090');
  const [error, setError] = useState('');
  
  // Statystyki
  const [pointCount, setPointCount] = useState(0);
  const [imageData, setImageData] = useState(null);
  const [availableTopics, setAvailableTopics] = useState([]);
  
  // Konfiguracja Chmury
  const [pointSize, setPointSize] = useState(0.05);
  const [colorMode, setColorMode] = useState('distance'); // 'distance' | 'height'
  const [rotationZ, setRotationZ] = useState(180); // Obrót w osi Z (domyślnie 180)
  const [decay, setDecay] = useState(true); // CZY BUDOWAĆ MAPĘ (Persistence)?
  
  // Wybór topiców
  const [selectedPointCloudTopic, setSelectedPointCloudTopic] = useState('/point_cloud_1');
  const [selectedImageTopic, setSelectedImageTopic] = useState('/camera/image_raw');
  
  // Refs (Three.js & ROS)
  const mountRef = useRef(null);
  const canvasRef = useRef(null);
  const pointCloudRef = useRef(null);
  const rosRef = useRef(null);
  
  // BUFOR NA PUNKTY (Dla efektu Mapy)
  // Przechowujemy do 60,000 punktów w pamięci (ok. 2-3 sekundy skanowania w wysokiej rozdzielczości lub dłużej w niskiej)
  const MAX_POINTS = 60000;
  const bufferIndexRef = useRef(0); // Gdzie aktualnie piszemy w buforze

  // --- STYLE ---
  const styles = {
    container: { display: 'flex', flexDirection: 'column', height: '100vh', background: '#0b0c10', color: '#c5c6c7', fontFamily: 'Segoe UI, sans-serif' },
    header: { background: '#1f2833', padding: '15px', borderBottom: '1px solid #45a29e', display: 'flex', gap: '20px', alignItems: 'center' },
    panelTitle: { color: '#66fcf1', margin: 0, fontSize: '20px', fontWeight: 'bold' },
    controlGroup: { display: 'flex', gap: '10px' },
    input: { padding: '8px', borderRadius: '4px', border: 'none', background: '#c5c6c7', color: '#000' },
    button: { padding: '8px 20px', borderRadius: '4px', border: 'none', fontWeight: 'bold', cursor: 'pointer', color: '#fff' },
    btnConnect: { background: '#45a29e' },
    btnDisconnect: { background: '#b80000' },
    content: { display: 'flex', flex: 1, overflow: 'hidden' },
    sidebar: { width: '320px', background: '#1f2833', padding: '15px', display: 'flex', flexDirection: 'column', gap: '15px', overflowY: 'auto', borderRight: '1px solid #45a29e' },
    mainView: { flex: 1, position: 'relative', background: '#000' },
    label: { fontSize: '12px', color: '#66fcf1', marginBottom: '5px', display: 'block' },
    select: { width: '100%', padding: '8px', background: '#0b0c10', color: '#fff', border: '1px solid #45a29e', borderRadius: '4px' },
    imagePreview: { width: '100%', aspectRatio: '16/9', background: '#000', border: '1px solid #45a29e', marginTop: '5px', objectFit: 'contain' },
    overlayStats: { position: 'absolute', top: 10, left: 10, background: 'rgba(0,0,0,0.8)', padding: '10px', borderRadius: '4px', pointerEvents: 'none', fontSize: '12px', color: '#fff' }
  };

  // --- INICJALIZACJA THREE.JS ---
  useEffect(() => {
    if (!mountRef.current) return;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x000000); // Czerń dla lepszego kontrastu
    
    // Podłoga pomocnicza
    const gridHelper = new THREE.GridHelper(20, 20, 0x222222, 0x111111);
    scene.add(gridHelper);
    
    // Osie
    const axesHelper = new THREE.AxesHelper(1);
    scene.add(axesHelper);

    // Kamera
    const width = mountRef.current.clientWidth;
    const height = mountRef.current.clientHeight;
    const camera = new THREE.PerspectiveCamera(60, width / height, 0.1, 1000);
    camera.position.set(0, 10, 10); // Widok z góry pod kątem

    // Renderer
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    mountRef.current.appendChild(renderer.domElement);

    // Controls
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;

    // --- PRZYGOTOWANIE DUŻEGO BUFORA PUNKTÓW (MAPA) ---
    // Zamiast tworzyć nową geometrię co klatkę, tworzymy jedną ogromną na start
    const geometry = new THREE.BufferGeometry();
    
    // Inicjalizacja pustymi zerami
    const positions = new Float32Array(MAX_POINTS * 3);
    const colors = new Float32Array(MAX_POINTS * 3);
    
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    
    // Ustawiamy, żeby na początku nic nie rysował (count = 0)
    geometry.setDrawRange(0, 0);

    const material = new THREE.PointsMaterial({ 
      size: pointSize, 
      vertexColors: true,
      sizeAttenuation: true,
      transparent: true,
      opacity: 0.8
    });
    
    const points = new THREE.Points(geometry, material);
    // Domyślna korekta ROS -> ThreeJS (Obrót -90 X żeby położyć "ścianę" na podłogę)
    points.rotation.x = -Math.PI / 2; 
    
    scene.add(points);
    pointCloudRef.current = points;

    // Resize
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

  // --- AKTUALIZACJA USTAWIEŃ W CZASIE RZECZYWISTYM ---
  useEffect(() => {
    if (pointCloudRef.current) {
      pointCloudRef.current.material.size = pointSize;
      
      // Obrót w osi Z (względem robota) + stała korekta osi X (-PI/2) dla ROSa
      pointCloudRef.current.rotation.set(-Math.PI / 2, 0, (rotationZ * Math.PI) / 180);
      pointCloudRef.current.updateMatrix();
    }
    
    // Reset bufora jeśli wyłączymy "Mapę"
    if (!decay && pointCloudRef.current) {
         bufferIndexRef.current = 0;
         pointCloudRef.current.geometry.setDrawRange(0, 0);
    }
  }, [pointSize, rotationZ, decay]);


  // --- DEKODOWANIE ---
  const decodeBase64 = (base64String) => {
    const binaryString = window.atob(base64String);
    const len = binaryString.length;
    const bytes = new Uint8Array(len);
    for (let i = 0; i < len; i++) bytes[i] = binaryString.charCodeAt(i);
    return bytes;
  };

  // --- PRZETWARZANIE CHMURY (TRYB MAPY) ---
  const processPointCloud = (message) => {
    if (!pointCloudRef.current) return;

    try {
      const data = decodeBase64(message.data);
      const view = new DataView(data.buffer);
      
      const fields = message.fields;
      const xOff = fields.find(f => f.name === 'x')?.offset ?? 0;
      const yOff = fields.find(f => f.name === 'y')?.offset ?? 4;
      const zOff = fields.find(f => f.name === 'z')?.offset ?? 8;
      const pointStep = message.point_step;
      const numPoints = message.width * message.height;

      const geometry = pointCloudRef.current.geometry;
      const posAttr = geometry.attributes.position;
      const colAttr = geometry.attributes.color;

      // Jeśli tryb MAPY jest wyłączony, resetujemy wskaźnik za każdym razem (czyścimy ekran)
      let currentIndex = decay ? bufferIndexRef.current : 0;

      for (let i = 0; i < numPoints; i++) {
        const offset = i * pointStep;
        if (offset + 12 > data.length) break;

        const x = view.getFloat32(offset + xOff, true);
        const y = view.getFloat32(offset + yOff, true);
        const z = view.getFloat32(offset + zOff, true);

        // Filtrowanie śmieci (0,0,0) i nieskończoności
        if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
        if (Math.abs(x) < 0.01 && Math.abs(y) < 0.01) continue; 

        // Zapisz pozycję do bufora
        posAttr.setXYZ(currentIndex, x, y, z);

        // KOLOROWANIE
        let r, g, b;
        
        if (colorMode === 'distance') {
            // ODLEGŁOŚĆ: Bliżej = Czerwony, Dalej = Niebieski
            const dist = Math.sqrt(x*x + y*y);
            const maxDist = 10.0; // metry
            const t = Math.min(1, dist / maxDist);
            
            // Prosty gradient HSL->RGB logic (Odwrócony: 0=Czerwony, 1=Niebieski)
            // Blisko (0) -> Czerwony/Żółty
            // Daleko (1) -> Niebieski/Fiolet
            r = 1.0 - t;
            g = 1.0 - t * 1.5; if(g<0) g=0;
            b = t;
        } else {
            // WYSOKOŚĆ (Z)
            const minZ = -1; const maxZ = 2;
            let t = (z - minZ) / (maxZ - minZ);
            t = Math.max(0, Math.min(1, t));
            r = t; g = t; b = t; // Skala szarości/wysokości
        }
        
        colAttr.setXYZ(currentIndex, r, g, b);

        // Przesuń wskaźnik (Ring Buffer)
        currentIndex++;
        if (currentIndex >= MAX_POINTS) currentIndex = 0;
      }

      // Zapisz gdzie skończyliśmy
      bufferIndexRef.current = currentIndex;

      // Poinformuj GPU o zmianach
      posAttr.needsUpdate = true;
      colAttr.needsUpdate = true;

      // Aktualizuj ile punktów rysować
      // Jeśli mapa jest pełna, rysujemy wszystko (MAX_POINTS), jeśli nie, to tyle ile zebraliśmy
      if (decay) {
          // Raz zapełniony bufor rysujemy cały czas
          if (currentIndex < bufferIndexRef.current) { 
               // Jeśli zawinęliśmy (currentIndex < stary index), to znaczy że bufor jest pełny
               geometry.setDrawRange(0, MAX_POINTS);
          } else {
               // Tutaj uproszczenie: zawsze rysujemy max jak już się trochę nazbiera
               // Ale na początku rysujemy tyle ile jest.
               // Dla prostoty: ustawiamy drawRange na MAX, bo puste są 0,0,0
               geometry.setDrawRange(0, MAX_POINTS);
          }
      } else {
          geometry.setDrawRange(0, currentIndex);
      }
      
      setPointCount(decay ? MAX_POINTS : currentIndex);

    } catch (err) {
      console.error(err);
    }
  };

  // --- PRZETWARZANIE OBRAZU (NAPRAWIONE) ---
  const processImage = (message) => {
    try {
        const width = message.width;
        const height = message.height;
        const canvas = canvasRef.current;
        if(!canvas) return;

        // Ustaw wymiary tylko jeśli się zmieniły (optymalizacja)
        if (canvas.width !== width) canvas.width = width;
        if (canvas.height !== height) canvas.height = height;
        
        const ctx = canvas.getContext('2d');
        const imgData = ctx.createImageData(width, height);
        
        // Dekoduj Base64
        const raw = window.atob(message.data);
        const rawLength = raw.length;
        const data = new Uint8Array(rawLength);
        for(let i = 0; i < rawLength; i++) data[i] = raw.charCodeAt(i);

        // Obsługa formatów
        const isBGR = message.encoding.includes('bgr');
        const isMono = message.encoding.includes('mono');
        
        let ptr = 0; // wskaźnik w danych wejściowych
        
        for(let i = 0; i < width * height; i++) {
            let r, g, b;
            
            if (isMono) {
                const val = data[ptr++];
                r = g = b = val;
            } else {
                // Zakładamy 3 kanały (rgb8 lub bgr8)
                const c1 = data[ptr++];
                const c2 = data[ptr++];
                const c3 = data[ptr++];
                
                if (isBGR) { r = c3; g = c2; b = c1; }
                else       { r = c1; g = c2; b = c3; }
            }

            imgData.data[i*4] = r;     // R
            imgData.data[i*4+1] = g;   // G
            imgData.data[i*4+2] = b;   // B
            imgData.data[i*4+3] = 255; // Alpha
        }
        
        ctx.putImageData(imgData, 0, 0);
        setImageData(canvas.toDataURL());
    } catch(e) {
        console.error("Błąd obrazu:", e);
    }
  };

  // --- ŁĄCZENIE Z ROS ---
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

        // Subskrypcja Chmury
        const pcSub = new ROSLIB.Topic({ ros, name: selectedPointCloudTopic, messageType: 'sensor_msgs/PointCloud2' });
        pcSub.subscribe(processPointCloud);
        
        // Subskrypcja Obrazu
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
      {/* 1. PASEK GÓRNY */}
      <div style={styles.header}>
        <div style={{flex:1}}>
             <h2 style={styles.panelTitle}>📡 ROS2 LiDAR Mapper</h2>
             <span style={{fontSize:'12px', color:'#888'}}>v3.0 Ultimate</span>
        </div>
        
        <div style={styles.controlGroup}>
          <input value={rosUrl} onChange={e=>setRosUrl(e.target.value)} style={styles.input} disabled={connected} />
          <button onClick={connected ? disconnect : connect} style={{...styles.button, ...(connected ? styles.btnDisconnect : styles.btnConnect)}}>
            {connected ? 'ROZŁĄCZ' : 'POŁĄCZ'}
          </button>
        </div>
      </div>

      <div style={styles.content}>
        
        {/* 2. LEWY PANEL */}
        <div style={styles.sidebar}>
          
          {/* Sekcja Obrazu */}
          <div>
            <span style={styles.label}>KAMERA</span>
            <select style={styles.select} value={selectedImageTopic} onChange={e=>setSelectedImageTopic(e.target.value)}>
                {availableTopics.filter(t=>t.type.includes('Image')).map(t=><option key={t.name}>{t.name}</option>)}
            </select>
            <div style={styles.imagePreview}>
               {imageData ? <img src={imageData} style={{width:'100%', height:'100%', objectFit:'contain'}} /> : <div style={{textAlign:'center', paddingTop:'20%', color:'#444'}}>NO SIGNAL</div>}
            </div>
            <canvas ref={canvasRef} style={{display:'none'}} />
          </div>

          <hr style={{borderColor:'#333', width:'100%', opacity: 0.3}} />

          {/* Sekcja Chmury */}
          <div>
            <span style={styles.label}>LIDAR / CHMURA</span>
            <select style={styles.select} value={selectedPointCloudTopic} onChange={e=>setSelectedPointCloudTopic(e.target.value)} style={{marginBottom:'15px', ...styles.select}}>
               {availableTopics.filter(t=>t.type.includes('Point')).map(t=><option key={t.name}>{t.name}</option>)}
            </select>

            {/* SUWAKI */}
            <div style={{background: '#151515', padding: '10px', borderRadius: '5px'}}>
                <span style={styles.label}>Rozmiar punktu: {pointSize}</span>
                <input type="range" min="0.01" max="0.3" step="0.01" value={pointSize} onChange={e => setPointSize(parseFloat(e.target.value))} style={{width: '100%'}} />
                
                <span style={{...styles.label, marginTop: '10px'}}>Obrót Z (dla 180°): {rotationZ}°</span>
                <input type="range" min="0" max="360" step="1" value={rotationZ} onChange={e => setRotationZ(parseFloat(e.target.value))} style={{width: '100%'}} />
            </div>

            {/* CHECKBOXY */}
            <div style={{marginTop:'15px', display:'flex', flexDirection:'column', gap:'10px'}}>
                <label style={{cursor:'pointer', display:'flex', alignItems:'center', gap:'10px'}}>
                    <input type="checkbox" checked={decay} onChange={e => setDecay(e.target.checked)} />
                    <span style={{color: decay ? '#66fcf1' : '#888'}}>🗺️ Tryb MAPY (Pamięć)</span>
                </label>
                
                <span style={styles.label}>Tryb Koloru:</span>
                <select style={styles.select} value={colorMode} onChange={e => setColorMode(e.target.value)}>
                    <option value="distance">🌈 Odległość (Radar)</option>
                    <option value="height">🏔️ Wysokość</option>
                </select>
            </div>

          </div>
        </div>

        {/* 3. WIDOK 3D */}
        <div style={styles.mainView} ref={mountRef}>
           <div style={styles.overlayStats}>
              Aktywne punkty: {pointCount.toLocaleString()}<br/>
              Tryb: {decay ? 'Budowanie mapy' : 'Skan na żywo'}
           </div>
        </div>

      </div>
    </div>
  );
};

export default RosViewer;
