import React, { useState, useEffect, useRef } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import ROSLIB from 'roslib';

const RosViewer = () => {
  // ===== STANY =====
  const [connected, setConnected] = useState(false);
  const [rosUrl, setRosUrl] = useState('ws://192.168.1.100:9090');
  const [error, setError] = useState('');

  const [availableTopics, setAvailableTopics] = useState([]);
  const [selectedPointCloudTopic, setSelectedPointCloudTopic] = useState('/point_cloud_1');
  const [selectedImageTopic, setSelectedImageTopic] = useState('/camera/image_raw');

  const [pointCount, setPointCount] = useState(0);
  const [imageData, setImageData] = useState(null);

  const [pointSize, setPointSize] = useState(0.05);
  const [freeze, setFreeze] = useState(false);
  const [maxHz, setMaxHz] = useState(10);

  // ===== REFY =====
  const mountRef = useRef(null);
  const canvasRef = useRef(null);
  const pointCloudRef = useRef(null);
  const rosRef = useRef(null);
  const rendererRef = useRef(null);
  const controlsRef = useRef(null);
  const lastUpdateRef = useRef(0);

  // ===== STYLE (BEZ ZMIAN WYGLĄDU) =====
  const styles = {
    container: { display: 'flex', flexDirection: 'column', height: '100vh', background: '#1a1a2e', color: 'white' },
    header: { background: '#16213e', padding: '15px', display: 'flex', gap: '20px', alignItems: 'center' },
    panelTitle: { color: '#4a9eff', margin: 0 },
    input: { padding: '8px', background: '#0f3460', color: 'white', border: '1px solid #444' },
    button: { padding: '8px 16px', border: 'none', color: 'white', cursor: 'pointer' },
    btnConnect: { background: '#16a085' },
    btnDisconnect: { background: '#c0392b' },
    content: { display: 'flex', flex: 1 },
    sidebar: { width: '300px', background: '#111', padding: '15px', display: 'flex', flexDirection: 'column', gap: '15px' },
    mainView: { flex: 1, position: 'relative', background: '#000' },
    select: { width: '100%', padding: '6px', background: '#222', color: '#fff' },
    imagePreview: { width: '100%', aspectRatio: '16/9', background: '#000' },
    label: { fontSize: '12px', color: '#aaa' },
    settingRow: { display: 'flex', justifyContent: 'space-between', alignItems: 'center' }
  };

  // ===== THREE.JS =====
  useEffect(() => {
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x111111);

    const camera = new THREE.PerspectiveCamera(
      75,
      mountRef.current.clientWidth / mountRef.current.clientHeight,
      0.1,
      1000
    );
    camera.position.set(6, 6, 8);

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(mountRef.current.clientWidth, mountRef.current.clientHeight);
    mountRef.current.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controlsRef.current = controls;

    scene.add(new THREE.GridHelper(20, 20));
    scene.add(new THREE.AxesHelper(2));

    const geometry = new THREE.BufferGeometry();
    const material = new THREE.PointsMaterial({
      size: pointSize,
      vertexColors: true,
      sizeAttenuation: true
    });

    const points = new THREE.Points(geometry, material);
    points.rotation.x = Math.PI; // 🔴 OBRÓT 180°
    scene.add(points);
    pointCloudRef.current = points;

    const animate = () => {
      requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
    };
    animate();

    return () => {
      renderer.dispose();
    };
  }, []);

  // ===== BASE64 =====
  const decodeBase64 = (base64) => {
    const bin = atob(base64);
    const bytes = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
    return bytes;
  };

  // ===== POINT CLOUD =====
  const processPointCloud = (msg) => {
    const now = performance.now();
    if (freeze) return;
    if (now - lastUpdateRef.current < 1000 / maxHz) return;
    lastUpdateRef.current = now;

    const data = decodeBase64(msg.data);
    const view = new DataView(data.buffer);

    const xOff = msg.fields.find(f => f.name === 'x')?.offset ?? 0;
    const yOff = msg.fields.find(f => f.name === 'y')?.offset ?? 4;
    const zOff = msg.fields.find(f => f.name === 'z')?.offset ?? 8;

    const positions = [];
    const colors = [];

    for (let i = 0; i < msg.width * msg.height; i++) {
      const o = i * msg.point_step;
      if (o + 12 > data.length) break;

      const x = view.getFloat32(o + xOff, true);
      const y = view.getFloat32(o + yOff, true);
      const z = view.getFloat32(o + zOff, true);

      if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;

      positions.push(x, y, z);

      const d = Math.sqrt(x*x + y*y + z*z);
      const t = Math.min(d / 10, 1);
      colors.push(1 - t, 0.2, t);
    }

    const g = pointCloudRef.current.geometry;
    g.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    g.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
    g.computeBoundingSphere();

    setPointCount(positions.length / 3);
  };

  // ===== IMAGE =====
  const processImage = (msg) => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    canvas.width = msg.width;
    canvas.height = msg.height;

    const img = ctx.createImageData(msg.width, msg.height);
    const data = decodeBase64(msg.data);

    if (msg.encoding === 'rgb8') {
      for (let i = 0; i < msg.width * msg.height; i++) {
        img.data[i*4]   = data[i*3];
        img.data[i*4+1] = data[i*3+1];
        img.data[i*4+2] = data[i*3+2];
        img.data[i*4+3] = 255;
      }
    } else if (msg.encoding === 'bgr8') {
      for (let i = 0; i < msg.width * msg.height; i++) {
        img.data[i*4]   = data[i*3+2];
        img.data[i*4+1] = data[i*3+1];
        img.data[i*4+2] = data[i*3];
        img.data[i*4+3] = 255;
      }
    }

    ctx.putImageData(img, 0, 0);
    setImageData(canvas.toDataURL());
  };

  // ===== ROS =====
  const connect = () => {
    const ros = new ROSLIB.Ros({ url: rosUrl });

    ros.on('connection', () => {
      setConnected(true);
      ros.getTopics(r =>
        setAvailableTopics(r.topics.map((n, i) => ({ name: n, type: r.types[i] })))
      );

      new ROSLIB.Topic({
        ros, name: selectedPointCloudTopic, messageType: 'sensor_msgs/PointCloud2'
      }).subscribe(processPointCloud);

      new ROSLIB.Topic({
        ros, name: selectedImageTopic, messageType: 'sensor_msgs/Image'
      }).subscribe(processImage);

      rosRef.current = ros;
    });

    ros.on('error', () => setError('Błąd połączenia'));
  };

  const disconnect = () => {
    rosRef.current?.close();
    setConnected(false);
  };

  // ===== RENDER =====
  return (
    <div style={styles.container}>
      <div style={styles.header}>
        <h2 style={styles.panelTitle}>🚀 ROS Viewer</h2>
        <input style={styles.input} value={rosUrl} onChange={e => setRosUrl(e.target.value)} />
        <button
          style={{ ...styles.button, ...(connected ? styles.btnDisconnect : styles.btnConnect) }}
          onClick={connected ? disconnect : connect}
        >
          {connected ? 'Rozłącz' : 'Połącz'}
        </button>
      </div>

      <div style={styles.content}>
        <div style={styles.sidebar}>
          <div>
            <div style={styles.label}>Image topic</div>
            <select style={styles.select} value={selectedImageTopic} onChange={e => setSelectedImageTopic(e.target.value)}>
              {availableTopics.filter(t => t.type.includes('Image')).map(t => <option key={t.name}>{t.name}</option>)}
            </select>
            <div style={styles.imagePreview}>
              {imageData && <img src={imageData} style={{ width: '100%' }} />}
            </div>
            <canvas ref={canvasRef} style={{ display: 'none' }} />
          </div>

          <div>
            <div style={styles.settingRow}>
              <span style={styles.label}>Rozmiar punktów</span>
              <input type="range" min="0.01" max="0.3" step="0.01"
                value={pointSize} onChange={e => setPointSize(+e.target.value)} />
            </div>

            <div style={styles.settingRow}>
              <span style={styles.label}>STOP / MAPA</span>
              <input type="checkbox" checked={freeze} onChange={e => setFreeze(e.target.checked)} />
            </div>
          </div>
        </div>

        <div ref={mountRef} style={styles.mainView}>
          <div style={{ position: 'absolute', top: 10, left: 10 }}>
            Punkty: {pointCount}
          </div>
        </div>
      </div>
    </div>
  );
};

export default RosViewer;
