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
  const [imageData, setImageData] = useState(null);
  const [availableTopics, setAvailableTopics] = useState([]);
  
  // Konfiguracja widoku
  const [pointSize, setPointSize] = useState(0.05);
  const [colorMode, setColorMode] = useState('distance'); // 'distance' | 'height' | 'intensity' | 'solid'
  const [flipAxes, setFlipAxes] = useState(true);
  const [rotate180Z, setRotate180Z] = useState(true); // Obrót 180 stopni wokół osi Z
  const [freezeFrame, setFreezeFrame] = useState(false); // Zatrzymanie aktualizacji
  const [updateRate, setUpdateRate] = useState(1); // 1 = każda ramka, 2 = co druga, itd.
  const [updateCounter, setUpdateCounter] = useState(0);
  
  // Zakresy do kolorowania
  const [distanceRange, setDistanceRange] = useState({ min: 0, max: 20 });
  const [heightRange, setHeightRange] = useState({ min: -2, max: 3 });
  
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
  const sceneRef = useRef(null);
  const frameCounterRef = useRef(0);

  // --- STYLE ---
  const styles = {
    container: { display: 'flex', flexDirection: 'column', height: '100vh', background: '#1a1a2e', color: 'white', fontFamily: 'Segoe UI, sans-serif' },
    header: { background: '#16213e', padding: '15px', borderBottom: '1px solid #2a2a4e', display: 'flex', flexWrap: 'wrap', gap: '20px', alignItems: 'center' },
    panelTitle: { color: '#4a9eff', margin: 0, fontSize: '18px', fontWeight: '600' },
    controlGroup: { display: 'flex', gap: '10px', alignItems: 'center' },
    input: { padding: '8px', borderRadius: '4px', border: '1px solid #444', background: '#0f3460', color: 'white' },
    button: { padding: '8px 16px', borderRadius: '4px', border: 'none', fontWeight: 'bold', cursor: 'pointer', color: 'white', transition: 'all 0.3s' },
    btnConnect: { background: '#16a085' },
    btnDisconnect: { background: '#c0392b' },
    btnFreeze: { background: '#f39c12' },
    content: { display: 'flex', flex: 1, overflow: 'hidden' },
    sidebar: { width: '320px', background: '#111', padding: '15px', display: 'flex', flexDirection: 'column', gap: '15px', overflowY: 'auto' },
    mainView: { flex: 1, position: 'relative', background: '#000' },
    settingRow: { display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '10px' },
    label: { fontSize: '13px', color: '#aaa' },
    select: { width: '100%', padding: '6px', background: '#222', color: '#fff', border: '1px solid #444', borderRadius: '4px' },
    imagePreview: { width: '100%', aspectRatio: '16/9', background: '#000', border: '1px solid #333', marginTop: '5px', objectFit: 'contain' },
    overlayStats: { position: 'absolute', top: 10, left: 10, background: 'rgba(0,0,0,0.7)', padding: '10px', borderRadius: '4px', pointerEvents: 'none', fontSize: '12px' },
    rangeInputs: { display: 'flex', gap: '10px', marginTop: '5px' },
    smallInput: { width: '70px', padding: '4px', background: '#222', color: 'white', border: '1px solid #444', borderRadius: '4px' }
  };

  // --- INICJALIZACJA THREE.JS ---
  useEffect(() => {
    if (!mountRef.current) return;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x111111);
    sceneRef.current = scene;
    
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
    
    if (flipAxes) {
      points.rotation.x = -Math.PI / 2;
    }
    
    if (rotate180Z) {
      points.rotation.z = Math.PI;
    }
    
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
      pointCloudRef.current.rotation.x = flipAxes ? -Math.PI / 2 : 0;
      pointCloudRef.current.rotation.z = rotate180Z ? Math.PI : 0;
      pointCloudRef.current.material.needsUpdate = true;
    }
  }, [pointSize, flipAxes, rotate180Z]);

  // --- PRZETWARZANIE CHMURY PUNKTÓW ---
  const decodeBase64 = (base64String) => {
    const binaryString = window.atob(base64String);
    const len = binaryString.length;
    const bytes = new Uint8Array(len);
    for (let i = 0; i < len; i++) bytes[i] = binaryString.charCodeAt(i);
    return bytes;
  };

  const processPointCloud = (message) => {
    try {
      // Kontrola aktualizacji
      frameCounterRef.current++;
      if (freezeFrame || (frameCounterRef.current % updateRate !== 0)) {
        return;
      }

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
      
      // Zmienne do automatycznego wykrywania zakresów
      let minDist = Infinity;
      let maxDist = -Infinity;
      let minHeight = Infinity;
      let maxHeight = -Infinity;

      // Pierwsze przejście - zbieranie danych dla zakresów
      for (let i = 0; i < numPoints; i++) {
        const offset = i * pointStep;
        if (offset + 12 > data.length) break;

        const x = view.getFloat32(offset + xOff, true);
        const y = view.getFloat32(offset + yOff, true);
        const z = view.getFloat32(offset + zOff, true);

        if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
        
        const distanceXY = Math.sqrt(x * x + y * y);
        minDist = Math.min(minDist, distanceXY);
        maxDist = Math.max(maxDist, distanceXY);
        minHeight = Math.min(minHeight, z);
        maxHeight = Math.max(maxHeight, z);
      }

      // Aktualizacja zakresów (z delikatnym wygładzeniem)
      if (maxDist > minDist) {
        setDistanceRange(prev => ({
          min: prev.min * 0.9 + minDist * 0.1,
          max: prev.max * 0.9 + maxDist * 0.1
        }));
      }
      
      if (maxHeight > minHeight) {
        setHeightRange(prev => ({
          min: prev.min * 0.9 + minHeight * 0.1,
          max: prev.max * 0.9 + maxHeight * 0.1
        }));
      }

      // Drugie przejście - przetwarzanie punktów z kolorowaniem
      for (let i = 0; i < numPoints; i++) {
        const offset = i * pointStep;
        if (offset + 12 > data.length) break;

        const x = view.getFloat32(offset + xOff, true);
        const y = view.getFloat32(offset + yOff, true);
        const z = view.getFloat32(offset + zOff, true);

        if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
        if (x === 0 && y === 0 && z === 0) continue;

        positions.push(x, y, z);

        const distanceXY = Math.sqrt(x * x + y * y);
        const angle = Math.atan2(y, x);
        
        // Różne tryby kolorowania
        if (colorMode === 'solid') {
          colors.push(1, 1, 1); // Biały
        } else if (colorMode === 'distance') {
          // Kolorowanie wg odległości w płaszczyźnie XY
          let normDist = (distanceXY - distanceRange.min) / (distanceRange.max - distanceRange.min);
          normDist = Math.max(0, Math.min(1, normDist));
          
          // Niebieski (blisko) -> Zielony -> Czerwony (daleko)
          if (normDist < 0.33) {
            colors.push(0, normDist * 3, 1 - normDist * 3); // Niebieski do cyjan
          } else if (normDist < 0.66) {
            const t = (normDist - 0.33) * 3;
            colors.push(t, 1 - t, 0); // Cyjan do żółtego
          } else {
            const t = (normDist - 0.66) * 3;
            colors.push(1, 1 - t, 0); // Żółty do czerwonego
          }
        } else if (colorMode === 'height') {
          // Kolorowanie wg wysokości (Z)
          let normHeight = (z - heightRange.min) / (heightRange.max - heightRange.min);
          normHeight = Math.max(0, Math.min(1, normHeight));
          
          // Niebieski (nisko) -> Zielony -> Czerwony (wysoko)
          colors.push(normHeight, 1 - Math.abs(0.5 - normHeight) * 2, 1 - normHeight);
        } else if (colorMode === 'directional') {
          // Kolorowanie wg kierunku (kąta w płaszczyźnie XY)
          const normAngle = (angle + Math.PI) / (2 * Math.PI);
          
          // Koło kolorów HSV
          const hue = normAngle;
          const saturation = 0.8;
          const value = 0.9;
          
          // Konwersja HSV do RGB
          const i = Math.floor(hue * 6);
          const f = hue * 6 - i;
          const p = value * (1 - saturation);
          const q = value * (1 - f * saturation);
          const t = value * (1 - (1 - f) * saturation);
          
          let r, g, b;
          switch (i % 6) {
            case 0: r = value; g = t; b = p; break;
            case 1: r = q; g = value; b = p; break;
            case 2: r = p; g = value; b = t; break;
            case 3: r = p; g = q; b = value; break;
            case 4: r = t; g = p; b = value; break;
            case 5: r = value; g = p; b = q; break;
          }
          
          colors.push(r, g, b);
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
      setUpdateCounter(frameCounterRef.current);
    } catch (err) {
      console.error(err);
    }
  };

  // --- PRZETWARZANIE OBRAZU ---
  const processImage = (message) => {
    try {
      // Kontrola aktualizacji dla obrazu
      if (freezeFrame || (frameCounterRef.current % updateRate !== 0)) {
        return;
      }

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
          imgData.data[i * 4] =
          imgData.data[i * 4 + 1] =
          imgData.data[i * 4 + 2] = g;
          imgData.data[i * 4 + 3] = 255;
        }
      }

      ctx.putImageData(imgData, 0, 0);
      setImageData(canvas.toDataURL());
    } catch (e) {
      console.error(e);
    }
  };

  // --- ROS CONNECTION ---
  const connect = () => {
    try {
      const ros = new ROSLIB.Ros({ url: rosUrl });
      ros.on('connection', () => {
        setConnected(true);
        rosRef.current = { ros };
        
        ros.getTopics((res) => {
            const list = res.topics.map((n, i) => ({ name: n, type: res.types[i] }));
            setAvailableTopics(list);
        });

        const pcSub = new ROSLIB.Topic({ 
          ros, 
          name: selectedPointCloudTopic, 
          messageType: 'sensor_msgs/PointCloud2' 
        });
        pcSub.subscribe(processPointCloud);
        
        const imgSub = new ROSLIB.Topic({ 
          ros, 
          name: selectedImageTopic, 
          messageType: 'sensor_msgs/Image' 
        });
        imgSub.subscribe(processImage);
        
        rosRef.current.pcSub = pcSub;
        rosRef.current.imgSub = imgSub;
      });
      
      ros.on('error', e => setError('Błąd połączenia: ' + e.message));
      ros.on('close', () => {
        setConnected(false);
        setError('Połączenie zamknięte');
      });
    } catch (e) { 
      setError(e.message); 
    }
  };
  
  const disconnect = () => {
    if(rosRef.current?.ros) rosRef.current.ros.close();
    setConnected(false);
    setError('');
  };

  // --- ZAPIS MAPY ---
  const saveMapSnapshot = () => {
    if (!rendererRef.current || !sceneRef.current) return;
    
    const canvas = rendererRef.current.domElement;
    const link = document.createElement('a');
    link.download = `map-snapshot-${new Date().toISOString()}.png`;
    link.href = canvas.toDataURL('image/png');
    link.click();
  };

  // --- RESET WIDOKU ---
  const resetView = () => {
    if (controlsRef.current) {
      controlsRef.current.reset();
    }
  };

  return (
    <div style={styles.container}>
      {/* 1. GÓRNY PASEK */}
      <div style={styles.header}>
        <h2 style={styles.panelTitle}>🚀 ROS2 Pro Viewer - Enhanced</h2>
        <div style={styles.controlGroup}>
          <input value={rosUrl} onChange={e=>setRosUrl(e.target.value)} style={styles.input} disabled={connected} />
          <button onClick={connected ? disconnect : connect} 
                  style={{...styles.button, ...(connected ? styles.btnDisconnect : styles.btnConnect)}}>
            {connected ? 'Rozłącz' : 'Połącz'}
          </button>
          <button onClick={() => setFreezeFrame(!freezeFrame)} 
                  style={{...styles.button, ...styles.btnFreeze}}>
            {freezeFrame ? '▶ Wznów' : '⏸ Zatrzymaj'}
          </button>
          <button onClick={saveMapSnapshot} 
                  style={{...styles.button, background: '#9b59b6'}}>
            📷 Zrzut mapy
          </button>
          <button onClick={resetView} 
                  style={{...styles.button, background: '#3498db'}}>
            🔄 Reset widoku
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
                <option value="">Wybierz topic obrazu</option>
                {availableTopics.filter(t=>t.type.includes('Image')).map(t=>
                  <option key={t.name} value={t.name}>{t.name}</option>
                )}
            </select>
            <div style={styles.imagePreview}>
               {imageData ? 
                 <img src={imageData} style={{width:'100%', height:'100%', objectFit:'contain'}} alt="Podgląd kamery" /> : 
                 <div style={{textAlign:'center', paddingTop:'20%', color:'#444'}}>Brak sygnału</div>
               }
            </div>
            <canvas ref={canvasRef} style={{display:'none'}} />
          </div>

          <hr style={{borderColor:'#333', width:'100%'}} />

          {/* Sekcja Ustawień Chmury 3D */}
          <div>
            <div style={styles.label}>CHMURA PUNKTÓW (PointCloud2)</div>
            <select style={styles.select} value={selectedPointCloudTopic} 
                    onChange={e=>setSelectedPointCloudTopic(e.target.value)}>
               <option value="">Wybierz topic chmury</option>
               {availableTopics.filter(t=>t.type.includes('Point')).map(t=>
                 <option key={t.name} value={t.name}>{t.name}</option>
               )}
            </select>

            {/* KONTROLA AKTUALIZACJI */}
            <div style={styles.settingRow}>
              <span style={styles.label}>Tempo aktualizacji: 1/{updateRate}</span>
            </div>
            <input 
              type="range" min="1" max="10" step="1" 
              value={updateRate} 
              onChange={e => setUpdateRate(parseInt(e.target.value))} 
              style={{width: '100%'}} 
            />

            {/* SUWAK WIELKOŚCI PUNKTÓW */}
            <div style={styles.settingRow}>
              <span style={styles.label}>Rozmiar punktów: {pointSize.toFixed(2)}</span>
            </div>
            <input 
              type="range" min="0.01" max="0.5" step="0.01" 
              value={pointSize} 
              onChange={e => setPointSize(parseFloat(e.target.value))} 
              style={{width: '100%'}} 
            />

            {/* TRYB KOLOROWANIA */}
            <div style={styles.settingRow}>
              <span style={styles.label}>Kolorowanie</span>
              <select style={styles.select} value={colorMode} onChange={e => setColorMode(e.target.value)}>
                <option value="distance">Odległość (XY)</option>
                <option value="height">Wysokość (Z)</option>
                <option value="directional">Kierunek (kąt XY)</option>
                <option value="solid">Jednolity (Biały)</option>
              </select>
            </div>

            {/* ZAKRESY KOLOROWANIA */}
            {colorMode === 'distance' && (
              <div>
                <div style={styles.label}>Zakres odległości (XY):</div>
                <div style={styles.rangeInputs}>
                  <input type="number" step="0.1" value={distanceRange.min.toFixed(1)} 
                         onChange={e => setDistanceRange({...distanceRange, min: parseFloat(e.target.value)})}
                         style={styles.smallInput} />
                  <span style={{lineHeight: '30px'}}>do</span>
                  <input type="number" step="0.1" value={distanceRange.max.toFixed(1)} 
                         onChange={e => setDistanceRange({...distanceRange, max: parseFloat(e.target.value)})}
                         style={styles.smallInput} />
                </div>
              </div>
            )}

            {colorMode === 'height' && (
              <div>
                <div style={styles.label}>Zakres wysokości (Z):</div>
                <div style={styles.rangeInputs}>
                  <input type="number" step="0.1" value={heightRange.min.toFixed(1)} 
                         onChange={e => setHeightRange({...heightRange, min: parseFloat(e.target.value)})}
                         style={styles.smallInput} />
                  <span style={{lineHeight: '30px'}}>do</span>
                  <input type="number" step="0.1" value={heightRange.max.toFixed(1)} 
                         onChange={e => setHeightRange({...heightRange, max: parseFloat(e.target.value)})}
                         style={styles.smallInput} />
                </div>
              </div>
            )}

            {/* OPCJE OBRACANIA */}
            <div style={{...styles.settingRow, marginTop:'15px'}}>
              <span style={styles.label}>Obróć Z-up → Y-up</span>
              <input type="checkbox" checked={flipAxes} onChange={e => setFlipAxes(e.target.checked)} />
            </div>
            
            <div style={styles.settingRow}>
              <span style={styles.label}>Obróć 180° wokół Z</span>
              <input type="checkbox" checked={rotate180Z} onChange={e => setRotate180Z(e.target.checked)} />
            </div>

            {/* STATYSTYKI */}
            <div style={{marginTop: '20px', padding: '10px', background: '#222', borderRadius: '4px'}}>
              <div style={styles.label}>Statystyki:</div>
              <div>Punkty: {pointCount.toLocaleString()}</div>
              <div>Ramka: {updateCounter}</div>
              <div>Tryb: {freezeFrame ? 'Zatrzymany' : 'Aktywny'}</div>
              <div>Aktualizacja: co {updateRate}. ramkę</div>
            </div>
          </div>
          
          <div style={{marginTop:'auto', fontSize:'11px', color:'#555'}}>
            <div>Sterowanie:</div>
            <div>LPM = Obrót, PPM = Przesuw, Rolka = Zoom</div>
            <div>Przyciski: Zatrzymaj/Wznów aktualizację</div>
            <div>📷 - Zapis zrzutu mapy</div>
          </div>
        </div>

        {/* WIDOK 3D */}
        <div style={styles.mainView} ref={mountRef}>
           <div style={styles.overlayStats}>
              <div style={{color: freezeFrame ? '#f39c12' : '#2ecc71', fontWeight: 'bold'}}>
                {freezeFrame ? '⏸ ZATRZYMANO' : '▶ AKTYWNY'}
              </div>
              <div>Punkty: {pointCount.toLocaleString()}</div>
              <div>Ramka: {updateCounter}</div>
              <div>Topic: {selectedPointCloudTopic || 'nie wybrano'}</div>
              <div>Kolorowanie: {colorMode === 'distance' ? 'Odległość' : 
                                colorMode === 'height' ? 'Wysokość' : 
                                colorMode === 'directional' ? 'Kierunek' : 'Jednolity'}</div>
           </div>
        </div>

      </div>
    </div>
  );
};

export default RosViewer;
