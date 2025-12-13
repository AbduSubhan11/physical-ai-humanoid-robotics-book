import React, { useState, useEffect, useRef } from 'react';
import axios from 'axios';
import { format } from 'date-fns';
import { API_BASE_URL } from '../config/api';


const ChatInterface = () => {
  const [messages, setMessages] = useState([]);
  const [inputMessage, setInputMessage] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isTyping, setIsTyping] = useState(false);
  const [conversationId, setConversationId] = useState(null);
  const [isOpen, setIsOpen] = useState(false);
  const messagesEndRef = useRef(null);

  // Initialize conversation when component mounts if not already initialized
  useEffect(() => {
    if (!conversationId) {
      startNewConversation();
    }
  }, []);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const startNewConversation = async () => {
    try {
      const response = await axios.post(`${API_BASE_URL}/chat/start`);
      setConversationId(response.data.conversation_id);
      setMessages([{
        id: 'welcome',
        role: 'assistant',
        content: 'Hello! I\'m your AI assistant for the robotics book. Ask me anything about robotics concepts, and I\'ll provide answers based on the book content.',
        timestamp: new Date()
      }]);
    } catch (error) {
      console.error('Error starting conversation:', error);
      setMessages([{
        id: 'error',
        role: 'assistant',
        content: 'Sorry, I\'m having trouble connecting. Please try again later.',
        timestamp: new Date()
      }]);
    }
  };

  const handleSendMessage = async (e) => {
    e.preventDefault();
    if (!inputMessage.trim() || isLoading || !conversationId) return;

    // Add user message to UI immediately
    const userMessage = {
      id: Date.now().toString(),
      role: 'user',
      content: inputMessage,
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputMessage('');
    setIsLoading(true);
    setIsTyping(true); // Show typing indicator

    try {
      // Send message to backend
      const response = await axios.post(
        `${API_BASE_URL}/chat/${conversationId}/message`,
        { message: inputMessage }
      );

      // Add AI response to messages
      const aiMessage = {
        id: Date.now().toString(),
        role: 'assistant',
        content: response.data.response,
        citations: response.data.citations,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, aiMessage]);
    } catch (error) {
      console.error('Error sending message:', error);

      // Check if it's a no-content error
      if (error.response?.data?.detail?.includes("no relevant information")) {
        const noContentMessage = {
          id: Date.now().toString(),
          role: 'assistant',
          content: "I couldn't find relevant information in the book content to answer your question. Please try rephrasing or ask about a different topic from the Physical AI & Humanoid Robotics book.",
          timestamp: new Date()
        };
        setMessages(prev => [...prev, noContentMessage]);
      } else {
        const errorMessage = {
          id: Date.now().toString(),
          role: 'assistant',
          content: 'Sorry, I encountered an error processing your request. Please try again.',
          timestamp: new Date()
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    } finally {
      setIsLoading(false);
      setIsTyping(false); // Hide typing indicator
    }
  };

  const formatCitations = (citations) => {
    if (!citations || citations.length === 0) return null;

    return (
      <div className="citations-container">
        <p className="citations-title">Sources:</p>
        <ul className="citations-list">
          {citations.map((citation, index) => (
            <li key={index} className="citation-item">
              {citation.section}
              {citation.page && `, Page ${citation.page}`}
              {citation.confidence && ` (Confidence: ${(citation.confidence * 100).toFixed(0)}%)`}
            </li>
          ))}
        </ul>
      </div>
    );
  };

  return (
    <>
      {/* Chat Icon Button */}
      <button className="chat-icon-button" onClick={() => setIsOpen(!isOpen)}>
        💬
      </button>

      {/* Chat Container - only show when isOpen is true */}
      <div className={`chat-container ${isOpen ? 'active' : ''}`}>
        {/* Chat Header */}
        <div className="chat-header">
          <h2 className="chat-title">Robotics Book Assistant</h2>
          <p className="chat-subtitle">Ask questions about Physical AI & Humanoid Robotics concepts</p>
          <button className="chat-close-button" onClick={() => setIsOpen(false)}>
            ×
          </button>
        </div>

      {/* Messages Container */}
      <div className="chat-messages-container">
        {messages.map((message) => (
          <div
            key={message.id}
            className={`message-wrapper ${message.role === 'user' ? 'user-message-wrapper' : 'assistant-message-wrapper'}`}
          >
            <div
              className={`message ${message.role === 'user' ? 'user-message' : 'assistant-message'}`}
            >
              <div className="message-content">{message.content}</div>
              {message.citations && formatCitations(message.citations)}
              <div className="message-timestamp">
                {format(message.timestamp, 'HH:mm')}
              </div>
            </div>
          </div>
        ))}
        {isTyping && (
          <div className="typing-indicator-wrapper">
            <div className="typing-indicator">
              <div className="typing-dots">
                <div className="typing-dot"></div>
                <div className="typing-dot"></div>
                <div className="typing-dot"></div>
              </div>
              <span className="typing-text">AI is thinking...</span>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      {/* Input Area */}
      <div className="chat-input-container">
        <form onSubmit={handleSendMessage} className="chat-input-form">
          <input
            type="text"
            value={inputMessage}
            onChange={(e) => setInputMessage(e.target.value)}
            placeholder="Ask a question about robotics..."
            className="chat-input"
            disabled={isLoading}
          />
          <button
            type="submit"
            disabled={isLoading || !inputMessage.trim()}
            className="chat-send-button"
          >
            {isLoading ? 'Sending...' : 'Send'}
          </button>
        </form>
        <div className="chat-disclaimer">
          Responses are based solely on the robotics book content
        </div>
      </div>
    </div>
    </>
  );
};

export default ChatInterface;