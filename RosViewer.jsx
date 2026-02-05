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
  const [updateRate, setUpdateRate] = useState(10); // Hz - ile razy na sekundę aktualizować
  const [downsample, setDownsample] = useState(1); // Co który punkt brać (1=wszystkie, 2=co drugi, etc)
  const [minDistance, setMinDistance] = useState(0); // Minimalna odległość punktu od sensora
  const [maxDistance, setMaxDistance] = useState(50); // Maksymalna odległość
  const [showStats, setShowStats] = useState(true);
  const [debugMode, setDebugMode] = useState(false);
  
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

  // --- DEKODOWANIE BASE64 ---
  const decodeBase64 = (base64String) => {
    const binaryString = window.atob(base64String);
    const len = binaryString.length;
    const bytes = new Uint8Array(len);
    for (let i = 0; i < len; i++) bytes[i] = binaryString.charCodeAt(i);
    return bytes;
  };

  // --- PRZETWARZANIE CHMURY Z THROTTLING I FILTRAMI ---
const processPointCloud = (message) => {
    try {
        // Throttling
        const now = Date.now();
        const minInterval = 1000 / updateRate;
        if (now - lastUpdateTimeRef.current < minInterval) return;
        lastUpdateTimeRef.current = now;

        // Get raw data
        const rawData = decodeBase64(message.data);
        const view = new DataView(rawData.buffer);
        
        const fields = message.fields;
        const pointStep = message.point_step;
        const numPoints = message.width * message.height;

        // CRITICAL: Locate field offsets (default to -1 if not found)
        const xOff = fields.find(f => f.name === 'x')?.offset ?? -1;
        const yOff = fields.find(f => f.name === 'y')?.offset ?? -1;
        const zOff = fields.find(f => f.name === 'z')?.offset ?? -1;
        const rgbOff = fields.find(f => f.name === 'rgb' || f.name === 'rgba')?.offset ?? -1;
        const intensityOff = fields.find(f => f.name === 'intensity')?.offset ?? -1;

        // DEBUG: Log message structure
        if (debugMode) {
            console.log('=== PointCloud2 DEBUG ===');
            console.log('Fields:', fields);
            console.log(`Points: ${numPoints}, Step: ${pointStep} bytes`);
            console.log(`Offsets - X:${xOff}, Y:${yOff}, Z:${zOff}, RGB:${rgbOff}, Intensity:${intensityOff}`);
            console.log('First 2 points raw data (hex):');
            for (let i=0; i<Math.min(2, numPoints); i++) {
                const hex = Array.from(new Uint8Array(rawData, i*pointStep, 16))
                    .map(b => b.toString(16).padStart(2, '0')).join(' ');
                console.log(`Point ${i}: ${hex}`);
            }
        }

        const newPositions = [];
        const newColors = [];

        let zeroCount = 0;
        let validCount = 0;
        let skippedCount = 0;

        for (let i = 0; i < numPoints; i += downsample) {
            const offset = i * pointStep;
            if (offset + 12 > rawData.length) break;

            // Read coordinates
            const x = (xOff >= 0) ? view.getFloat32(offset + xOff, true) : 0;
            const y = (yOff >= 0) ? view.getFloat32(offset + yOff, true) : 0;
            const z = (zOff >= 0) ? view.getFloat32(offset + zOff, true) : 0;

            // Count zero points for debug
            if (x === 0 && y === 0 && z === 0) {
                zeroCount++;
                // Skip zero points early to avoid processing
                if (debugMode && i < 10) {
                    console.warn(`Zero point at index ${i}, offset ${offset}`);
                }
                continue;
            }

            // Validate coordinates
            if (!isFinite(x) || !isFinite(y) || !isFinite(z)) {
                skippedCount++;
                continue;
            }

            // Distance filter
            const distance = Math.sqrt(x*x + y*y + z*z);
            if (distance < minDistance || distance > maxDistance) {
                filteredCount++;
                continue;
            }

            // COLOR PROCESSING - PRIORITIZE MESSAGE DATA
            let r = 1, g = 1, b = 1; // Default white

            if (rgbOff >= 0) {
                // Handle packed RGB/RGBA (Isaac Sim often uses BGR order)
                const packed = view.getUint32(offset + rgbOff, true);
                if (debugMode && i < 5) {
                    console.log(`Point ${i} packed RGB: 0x${packed.toString(16)}`);
                }
                // Extract B,G,R,A bytes (common ROS format[citation:3])
                b = ((packed >> 0) & 0xFF) / 255.0;
                g = ((packed >> 8) & 0xFF) / 255.0;
                r = ((packed >> 16) & 0xFF) / 255.0;
                // Alpha at >>24
            } else if (intensityOff >= 0) {
                // Use intensity for grayscale
                const intensity = view.getFloat32(offset + intensityOff, true);
                const norm = Math.min(Math.max(intensity, 0), 1);
                r = g = b = norm;
            } else {
                // Fallback to height/distance coloring
                if (colorMode === 'distance') {
                    const norm = Math.min(distance / maxDistance, 1);
                    r = 1 - norm;
                    g = norm * 0.5;
                    b = norm;
                } else if (colorMode === 'height') {
                    const minZ = -2, maxZ = 3;
                    let norm = (z - minZ) / (maxZ - minZ);
                    norm = Math.max(0, Math.min(1, norm));
                    r = norm;
                    g = 1 - Math.abs(0.5 - norm) * 2;
                    b = 1 - norm;
                }
            }

            newPositions.push(x, y, z);
            newColors.push(r, g, b);
            validCount++;
        }

        // Update geometry
        if (validCount > 0) {
            const geometry = pointCloudRef.current.geometry;
            geometry.setAttribute('position', 
                new THREE.Float32BufferAttribute(newPositions, 3));
            geometry.setAttribute('color', 
                new THREE.Float32BufferAttribute(newColors, 3));
            geometry.attributes.position.needsUpdate = true;
            geometry.attributes.color.needsUpdate = true;
        }

        // Update stats with critical info
        setPointCount(validCount);
        if (debugMode) {
            console.log(`Processed: ${validCount} valid, ${zeroCount} zero, ${skippedCount} skipped`);
            if (zeroCount > numPoints * 0.5) {
                console.error('WARNING: More than 50% points are (0,0,0)! Likely Isaac Sim issue[citation:1]');
            }
        }

    } catch (err) {
        console.error('Point cloud processing error:', err);
        if (debugMode) {
            console.error('Full message dump:', message);
        }
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
              <div style={styles.sectionTitle}>🐛 DEBUG</div>
              
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

              <div style={{marginTop:'8px', fontSize:'10px', color:'#888'}}>
                💡 Update Rate = ile razy/s aktualizować wizualizację<br/>
                💡 Downsampling = co który punkt brać (mniej = szybciej)<br/>
                💡 Filtr odległości = usuń punkty za blisko/daleko
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
