import React from 'react';
import Layout from '@theme/Layout';
import Chatbot from '../components/Chatbot';

function ChatbotPage() {
  return (
    <Layout title="Chat with the Book" description="Chat with the AI about the Physical AI & Humanoid Robotics book content.">
      <main>
        <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '100%' }}>
          <Chatbot />
        </div>
      </main>
    </Layout>
  );
}

export default ChatbotPage;
