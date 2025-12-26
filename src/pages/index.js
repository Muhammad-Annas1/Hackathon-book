import React, { useEffect } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import lottie from 'lottie-web';

export default function Home() {

  useEffect(() => {
    lottie.loadAnimation({
      container: document.getElementById('robot-lottie'),
      renderer: 'svg',
      loop: true,
      autoplay: true,
      path: '/lottie/Robot.json',
    });
  }, []);

  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="Learn ROS 2, NVIDIA Isaac, Digital Twins & Vision-Language-Action systems"
    >
      <main
        style={{
          fontFamily: "'Inter', sans-serif",
          overflowX: 'hidden',
          minHeight: '90vh',
          position: 'relative',
          background: '#1e0f3f',
          padding: '100px 10%',
        }}
      >
        {/* Background */}
        <div
          style={{
            position: 'absolute',
            inset: 0,
            zIndex: 0,
            background: 'radial-gradient(circle at top, #1e0f3f, #0d061f)',
          }}
        >
          <div className="particle"></div>
          <div className="particle"></div>
          <div className="particle"></div>
          <div className="particle"></div>
        </div>

        {/* Hero */}
        <section
          style={{
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'space-between',
            gap: '60px',
            position: 'relative',
            zIndex: 1,
            flexWrap: 'wrap',
          }}
        >
          {/* LEFT */}
          <div style={{ flex: 1, minWidth: '320px' }}>
            <h1 className="hero-heading">
              Physical AI & <br /> Humanoid Robotics
            </h1>

            <p className="hero-paragraph">
              Learn how intelligent humanoid robots perceive, reason, and act
              in the physical world using modern AI systems.
            </p>

            <div style={{ display: 'flex', gap: '20px', flexWrap: 'wrap' }}>
              <Link to="/docs/intro" className="primary-btn">
                📘 Read Book
              </Link>
              <Link to="/docs/getting-started" className="secondary-btn">
                🚀 Start Learning
              </Link>
            </div>
          </div>

          {/* RIGHT – PURPLE ROBOT */}
          <div className="robot-wrapper">
            <div id="robot-lottie" className="robot-lottie" />
            <div className="purple-overlay" /> {/* Chhota aur rounded overlay */}
          </div>
        </section>

        {/* Styles */}
        <style>{`
          /* Particles */
          .particle {
            position: absolute;
            width: 4px;
            height: 4px;
            background: #a855f7;
            border-radius: 50%;
            top: 50%;
            left: 50%;
            animation: particleAnim 6s linear infinite;
            opacity: 0.6;
          }

          @keyframes particleAnim {
            0% { transform: translate(0,0) scale(0.5); }
            50% { transform: translate(-40vw, -40vh) scale(1); }
            100% { transform: translate(0,0) scale(0.5); }
          }

          /* Heading */
          .hero-heading {
            font-size: 3.6rem;
            font-weight: 900;
            margin-bottom: 24px;
            color: #d8b4fe;
            text-shadow: 0 0 10px #a855f7, 0 0 24px #7c3aed;
          }

          .hero-paragraph {
            font-size: 1.25rem;
            color: #d8b4fe;
            margin-bottom: 40px;
            max-width: 520px;
            line-height: 1.7;
          }

          /* Buttons */
          .primary-btn {
            padding: 16px 34px;
            border-radius: 14px;
            background: linear-gradient(135deg, #a855f7, #7c3aed);
            color: white;
            text-decoration: none;
            box-shadow: 0 0 20px #a855f7;
            transition: 0.3s;
          }

          .secondary-btn {
            padding: 16px 34px;
            border-radius: 14px;
            border: 2px solid #a855f7;
            color: #a855f7;
            text-decoration: none;
            box-shadow: 0 0 10px #a855f7;
          }

          /* ROBOT */
          .robot-wrapper {
            position: relative;
            width: 420px;
            height: 420px;
            margin: auto;
          }

          .robot-lottie {
            width: 100%;
            height: 100%;
            position: relative;
            z-index: 1;
            filter: brightness(1.05) contrast(1.05);
          }

          /* 🔥 SMALL & ROUNDED PURPLE OVERLAY */
          .purple-overlay {
            position: absolute;
            top: 15%;
            left: 15%;
            width: 70%;
            height: 70%;
            background: linear-gradient(
              135deg,
              rgba(124,58,237,0.75),
              rgba(168,85,247,0.75)
            );
            mix-blend-mode: color;
            pointer-events: none;
            z-index: 2;
            border-radius: 50%; /* fully rounded */
            transition: 0.3s;
          }

          .robot-wrapper:hover .purple-overlay {
            background: linear-gradient(
              135deg,
              rgba(124,58,237,0.9),
              rgba(168,85,247,0.9)
            );
          }

          @media (max-width: 768px) {
            .hero-heading { font-size: 2.6rem; }
            .robot-wrapper { width: 300px; height: 300px; }
            .purple-overlay {
              top: 10%;
              left: 10%;
              width: 80%;
              height: 80%;
            }
          }
        `}</style>
      </main>
    </Layout>
  );
}
