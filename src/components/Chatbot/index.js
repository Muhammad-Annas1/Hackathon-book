import React, { useState, useRef, useEffect } from 'react';
import styles from './Chatbot.module.css';

const Chatbot = () => {
    const [messages, setMessages] = useState([]);
    const [input, setInput] = useState('');
    const [isLoading, setIsLoading] = useState(false);
    const messagesEndRef = useRef(null);

    const API_URL = process.env.NODE_ENV === 'production' 
                    ? 'https://your-production-backend.com/api/v1/chat' // TODO: Replace with actual production URL
                    : 'http://localhost:8000/api/v1/chat'; // Assuming FastAPI runs on 8000

    const scrollToBottom = () => {
        messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
    };

    useEffect(scrollToBottom, [messages]);

    const handleSendMessage = async (e) => {
        e.preventDefault();
        if (input.trim() === '') return;

        const userMessage = { id: Date.now(), text: input, sender: 'user' };
        setMessages((prevMessages) => [...prevMessages, userMessage]);
        setInput('');
        setIsLoading(true);

        try {
            const response = await fetch(API_URL, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ question: input }),
            });

            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }

            const data = await response.json();
            const botMessage = { id: Date.now() + 1, text: data.answer, sender: 'bot' };
            setMessages((prevMessages) => [...prevMessages, botMessage]);
        } catch (error) {
            console.error("Error sending message:", error);
            const errorMessage = { id: Date.now() + 1, text: "Sorry, I couldn't get a response. Please try again.", sender: 'bot' };
            setMessages((prevMessages) => [...prevMessages, errorMessage]);
        } finally {
            setIsLoading(false);
        }
    };

    return (
        <div className={styles.chatbotContainer}>
            <div className={styles.messagesContainer}>
                {messages.map((message) => (
                    <div key={message.id} className={`${styles.message} ${styles[message.sender]}`}>
                        {message.text}
                    </div>
                ))}
                {isLoading && (
                    <div className={`${styles.message} ${styles.bot}`}>
                        Thinking...
                    </div>
                )}
                <div ref={messagesEndRef} />
            </div>
            <form onSubmit={handleSendMessage} className={styles.inputContainer}>
                <input
                    type="text"
                    value={input}
                    onChange={(e) => setInput(e.target.value)}
                    placeholder="Ask me anything..."
                    className={styles.inputField}
                    disabled={isLoading}
                />
                <button type="submit" className={styles.sendButton} disabled={isLoading}>
                    Send
                </button>
            </form>
        </div>
    );
};

export default Chatbot;
