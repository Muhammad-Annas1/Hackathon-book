import React, { useState, useRef, useEffect } from 'react';

const Chatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    {
      role: 'assistant',
      content:
        "Hey there! 👋 I'm your guide to this book on Physical AI and Humanoid Robotics. Ask me anything, or select some text on the page and I'll help explain it!",
    },
  ]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  useEffect(() => {
    const handleSelectionChange = () => {
      const selection = window.getSelection().toString().trim();
      if (selection && selection.length > 10) {
        setSelectedText(selection);
      }
    };
    document.addEventListener('mouseup', handleSelectionChange);
    return () =>
      document.removeEventListener('mouseup', handleSelectionChange);
  }, []);

  const clearSelectedText = () => {
    setSelectedText('');
    window.getSelection().removeAllRanges();
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!input.trim()) return;

    setMessages((prev) => [...prev, { role: 'user', content: input }]);
    setInput('');
    setIsLoading(true);

    try {
      const response = await fetch(
        'http://localhost:8000/api/v1/chat',
        {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            question: input,
            context: selectedText || null,
          }),
        }
      );

      const data = await response.json();
      setMessages((prev) => [
        ...prev,
        { role: 'assistant', content: data.answer },
      ]);

      if (selectedText) clearSelectedText();
    } catch {
      setMessages((prev) => [
        ...prev,
        {
          role: 'assistant',
          content:
            "⚠️ Couldn't connect to backend. Please check FastAPI server.",
        },
      ]);
    } finally {
      setIsLoading(false);
    }
  };

  const styles = {
    container: {
      position: 'fixed',
      bottom: '24px',
      right: '24px',
      zIndex: 9999,
      fontFamily:
        '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif',
    },

    chatWindow: {
      width: '400px',
      height: '580px',
      background:
        'radial-gradient(circle at top, rgba(142,103,232,0.25), rgba(8,10,20,0.96) 45%)',
      backdropFilter: 'blur(22px)',
      borderRadius: '24px',
      boxShadow:
        '0 40px 90px rgba(142,103,232,0.45), inset 0 0 0 1px rgba(255,255,255,0.08)',
      display: 'flex',
      flexDirection: 'column',
      overflow: 'hidden',
      animation: 'slideUp 0.35s ease-out',
    },

    header: {
      padding: '16px 20px',
      background:
        'linear-gradient(135deg, #8e67e8, #4f46e5)',
      color: 'white',
      display: 'flex',
      justifyContent: 'space-between',
      alignItems: 'center',
    },

    headerTitle: {
      margin: 0,
      fontSize: '15px',
      fontWeight: '600',
      display: 'flex',
      alignItems: 'center',
      gap: '8px',
      letterSpacing: '0.4px',
    },

    closeButton: {
      background: 'rgba(255,255,255,0.2)',
      border: 'none',
      color: 'white',
      width: '36px',
      height: '36px',
      borderRadius: '50%',
      cursor: 'pointer',
      fontSize: '18px',
    },

    selectedTextBanner: {
      padding: '10px 16px',
      background:
        'linear-gradient(90deg, rgba(142,103,232,0.18), rgba(79,70,229,0.18))',
      borderBottom: '1px solid rgba(142,103,232,0.35)',
      display: 'flex',
      justifyContent: 'space-between',
      alignItems: 'center',
    },

    messagesArea: {
      flex: 1,
      padding: '18px',
      display: 'flex',
      flexDirection: 'column',
      gap: '14px',
      overflowY: 'auto',
    },

    message: (isUser) => ({
      alignSelf: isUser ? 'flex-end' : 'flex-start',
      background: isUser
        ? 'linear-gradient(135deg, #8e67e8, #4f46e5)'
        : 'rgba(255,255,255,0.08)',
      color: '#f9fafb',
      padding: '12px 16px',
      borderRadius: isUser
        ? '20px 20px 6px 20px'
        : '20px 20px 20px 6px',
      maxWidth: '85%',
      fontSize: '14px',
    }),

    inputForm: {
      padding: '16px',
      borderTop: '1px solid rgba(255,255,255,0.08)',
      display: 'flex',
      gap: '10px',
      background: 'rgba(6,8,18,0.95)',
    },

    input: {
      flex: 1,
      padding: '12px 18px',
      borderRadius: '26px',
      border: '1px solid rgba(142,103,232,0.45)',
      background: 'rgba(255,255,255,0.06)',
      color: '#fff',
      outline: 'none',
    },

    sendButton: {
      width: '48px',
      height: '48px',
      borderRadius: '50%',
      background:
        'linear-gradient(135deg, #8e67e8, #4f46e5)',
      border: 'none',
      color: 'white',
      fontSize: '18px',
      cursor: 'pointer',
    },

    /* 🤖 Floating AI Button (ICON CHANGED) */
    fabButton: {
      width: '70px',
      height: '70px',
      borderRadius: '50%',
      background:
        'linear-gradient(135deg, #8e67e8, #4f46e5)',
      color: 'white',
      border: 'none',
      boxShadow:
        '0 0 35px rgba(142,103,232,1)',
      cursor: 'pointer',
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center',
      fontSize: '32px',
    },
  };

  useEffect(() => {
    const style = document.createElement('style');
    style.innerHTML = `
      @keyframes slideUp {
        from { opacity: 0; transform: translateY(20px); }
        to { opacity: 1; transform: translateY(0); }
      }
    `;
    document.head.appendChild(style);
    return () => document.head.removeChild(style);
  }, []);

  return (
    <div style={styles.container}>
      {isOpen ? (
        <div style={styles.chatWindow}>
          <div style={styles.header}>
            <h4 style={styles.headerTitle}>🤖 AI Book Assistant</h4>
            <button
              style={styles.closeButton}
              onClick={() => setIsOpen(false)}
            >
              ×
            </button>
          </div>

          <div style={styles.messagesArea}>
            {messages.map((m, i) => (
              <div key={i} style={styles.message(m.role === 'user')}>
                {m.content}
              </div>
            ))}
            <div ref={messagesEndRef} />
          </div>

          <form onSubmit={handleSubmit} style={styles.inputForm}>
            <input
              value={input}
              onChange={(e) => setInput(e.target.value)}
              placeholder="Ask about robotics, AI, ROS…"
              style={styles.input}
            />
            <button type="submit" style={styles.sendButton}>
              ➤
            </button>
          </form>
        </div>
      ) : (
        <button
          style={styles.fabButton}
          onClick={() => setIsOpen(true)}
          title="Open AI Assistant"
        >
          🤖
        </button>
      )}
    </div>
  );
};

export default Chatbot;
