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
          padding: '80px 20px',
          textAlign: 'center',
          background: '#f9fafb',
          minHeight: '80vh',
        }}
      >
        {/* Hero Section */}
        <h1
          style={{
            fontSize: '3rem',
            fontWeight: 'bold',
            marginBottom: '20px',
            color: '#111827',
          }}
        >
          Welcome to <span style={{ color: '#4f46e5' }}>Physical AI & Humanoid Robotics</span>
        </h1>

        <p
          style={{
            fontSize: '1.2rem',
            maxWidth: '700px',
            color: '#4b5563',
            marginBottom: '40px',
          }}
        >
          A complete open-source book covering modern robotics: ROS 2, Digital Twin simulations,
          NVIDIA Isaac, Motor Control, VLA Systems, and everything you need to build real humanoids.
        </p>

        {/* Buttons */}
        <div style={{ display: 'flex', gap: '20px' }}>
          <a
            href="/docs/intro"
            style={{
              padding: '12px 25px',
              backgroundColor: '#4f46e5',
              color: 'white',
              borderRadius: '8px',
              textDecoration: 'none',
              fontSize: '1rem',
            }}
          >
            Start Learning
          </a>

          <a
            href="https://github.com/Muhammad-Annas1/Hackathon-book"
            target="_blank"
            style={{
              padding: '12px 25px',
              border: '2px solid #4f46e5',
              borderRadius: '8px',
              color: '#4f46e5',
              textDecoration: 'none',
              fontSize: '1rem',
            }}
          >
            View on GitHub
          </a>
        </div>

        {/* Footer Section */}
        <p
          style={{
            marginTop: '50px',
            color: '#6b7280',
            fontSize: '0.95rem',
          }}
        >
          Explore the documentation using the sidebar on the left.
        </p>
      </main>
    </Layout>
  );
}
