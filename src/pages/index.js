import React from 'react';
import Layout from '@theme/Layout';

export default function Home() {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="Learn ROS 2, NVIDIA Isaac, Digital Twins & Vision-Language-Action systems"
    >
      <main
        style={{
          display: 'flex',
          flexDirection: 'column',
          justifyContent: 'center',
          alignItems: 'center',
          padding: '100px 20px',
          textAlign: 'center',
          background: 'linear-gradient(135deg, #f0f7ff 0%, #ffffff 100%)', // ⭐ SOFT BACKGROUND
          minHeight: '85vh',
        }}
      >

        {/* Hero Section */}
        <h1
          style={{
            fontSize: '3.2rem',
            fontWeight: '800',
            marginBottom: '20px',
            color: '#111827',
            maxWidth: '900px',
            lineHeight: '1.2',
          }}
        >
          Build the Future with{' '}
          <span style={{ color: '#4f46e5' }}>Physical AI & Humanoid Robotics</span>
        </h1>

        <p
          style={{
            fontSize: '1.25rem',
            maxWidth: '750px',
            color: '#4b5563',
            marginBottom: '45px',
            lineHeight: '1.7',
          }}
        >
          Learn everything from ROS 2, Digital Twin Simulations, NVIDIA Isaac, Motor Control,
          Vision-Language-Action systems — step by step.
        </p>

        {/* Buttons */}
        <div style={{ display: 'flex', gap: '18px' }}>
          <a
            href="/docs/intro"
            style={{
              padding: '14px 28px',
              backgroundColor: '#4f46e5',
              color: 'white',
              borderRadius: '10px',
              textDecoration: 'none',
              fontSize: '1.05rem',
              fontWeight: '500',
              boxShadow: '0 4px 14px rgba(79, 70, 229, 0.3)',
              transition: 'all 0.25s',
            }}
            onMouseOver={(e) => (e.target.style.transform = 'scale(1.05)')}
            onMouseOut={(e) => (e.target.style.transform = 'scale(1)')}
          >
            🚀 Start Learning
          </a>

          <a
            href="https://github.com/Muhammad-Annas1/Hackathon-book"
            target="_blank"
            style={{
              padding: '14px 28px',
              border: '2px solid #4f46e5',
              borderRadius: '10px',
              color: '#4f46e5',
              textDecoration: 'none',
              fontSize: '1.05rem',
              fontWeight: '500',
              transition: 'all 0.25s',
            }}
            onMouseOver={(e) => {
              e.target.style.backgroundColor = '#eef2ff';
              e.target.style.transform = 'scale(1.05)';
            }}
            onMouseOut={(e) => {
              e.target.style.backgroundColor = 'transparent';
              e.target.style.transform = 'scale(1)';
            }}
          >
            ⭐ View on GitHub
          </a>
        </div>

        {/* Footer */}
        <p
          style={{
            marginTop: '55px',
            color: '#6b7280',
            fontSize: '1rem',
          }}
        >
          Use the sidebar to explore each module and build your robotics journey.
        </p>
      </main>
    </Layout>
  );
}
