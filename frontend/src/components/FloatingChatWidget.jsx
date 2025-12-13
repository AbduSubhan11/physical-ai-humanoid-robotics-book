import React, { useState, useEffect, useRef } from 'react';
import { format } from 'date-fns';
import { useChat } from './ChatContext';
import { API_BASE_URL } from '../config/api';

const FloatingChatWidget = () => {
  const { state, dispatch } = useChat();
  const messagesEndRef = useRef(null);
  const chatWindowRef = useRef(null);

  // Initialize conversation when component mounts and chat is opened
  useEffect(() => {
    if (state.isOpen && !state.conversationId) {
      startNewConversation();
    }
  }, [state.isOpen]);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [state.messages, state.isTyping]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const startNewConversation = async () => {
    try {
      const response = await fetch(`${API_BASE_URL}/chat/start`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
      });
      const data = await response.json();
      dispatch({ type: 'SET_CONVERSATION_ID', payload: data.conversation_id });
    } catch (error) {
      console.error('Error starting conversation:', error);
    }
  };

  const handleSendMessage = async (e) => {
    e.preventDefault();
    if (!state.inputMessage.trim() || state.isLoading || !state.conversationId) return;

    // Add user message to UI immediately
    const userMessage = {
      id: Date.now().toString(),
      role: 'user',
      content: state.inputMessage,
      timestamp: new Date()
    };

    dispatch({ type: 'ADD_MESSAGE', payload: userMessage });
    dispatch({ type: 'SET_INPUT_MESSAGE', payload: '' });
    dispatch({ type: 'SET_LOADING', payload: true });
    dispatch({ type: 'SET_TYPING', payload: true });

    try {
      // Send message to backend
      const response = await fetch(`${API_BASE_URL}/chat/${state.conversationId}/message`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ message: state.inputMessage })
      });

      const data = await response.json();

      // Add AI response to messages
      const aiMessage = {
        id: Date.now().toString(),
        role: 'assistant',
        content: data.response,
        citations: data.citations,
        timestamp: new Date()
      };

      dispatch({ type: 'ADD_MESSAGE', payload: aiMessage });
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        id: Date.now().toString(),
        role: 'assistant',
        content: 'Sorry, I encountered an error processing your request. Please try again.',
        timestamp: new Date()
      };
      dispatch({ type: 'ADD_MESSAGE', payload: errorMessage });
    } finally {
      dispatch({ type: 'SET_LOADING', payload: false });
      dispatch({ type: 'SET_TYPING', payload: false });
    }
  };

  const formatCitations = (citations) => {
    if (!citations || citations.length === 0) return null;

    return (
      <div className="mt-2 text-xs text-gray-600">
        <p className="font-semibold text-gray-700">Sources:</p>
        <ul className="list-disc list-inside text-xs">
          {citations.map((citation, index) => (
            <li key={index} className="break-words">
              {citation.section}
              {citation.page && `, Page ${citation.page}`}
              {citation.confidence && ` (Confidence: ${(citation.confidence * 100).toFixed(0)}%)`}
            </li>
          ))}
        </ul>
      </div>
    );
  };

  const toggleChat = () => {
    dispatch({ type: 'SET_IS_OPEN', payload: !state.isOpen });
  };

  const closeChat = () => {
    dispatch({ type: 'SET_IS_OPEN', payload: false });
  };

  return (
    <>
      {/* Floating Chat Button */}
      {!state.isOpen && (
        <button
          onClick={toggleChat}
          className="chatbot-button"
          aria-label="Open chat"
        >
          <svg xmlns="http://www.w3.org/2000/svg" className="chatbot-icon" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M8 12h.01M12 12h.01M16 12h.01M21 12c0 4.418-4.03 8-9 8a9.863 9.863 0 01-4.255-.949L3 20l1.395-3.72C3.512 15.042 3 13.574 3 12c0-4.418 4.03-8 9-8s9 3.582 9 8z" />
          </svg>
        </button>
      )}

      {/* Chat Window */}
      {state.isOpen && (
        <div
          ref={chatWindowRef}
          className="chatbot-window"
        >
          {/* Chat Header */}
          <div className="chatbot-header">
            <h3 className="chatbot-title">Robotics Book Assistant</h3>
            <button
              onClick={closeChat}
              className="chatbot-close-button"
            >
              <svg xmlns="http://www.w3.org/2000/svg" className="chatbot-close-icon" viewBox="0 0 20 20" fill="currentColor">
                <path fillRule="evenodd" d="M4.293 4.293a1 1 0 011.414 0L10 8.586l4.293-4.293a1 1 0 111.414 1.414L11.414 10l4.293 4.293a1 1 0 01-1.414 1.414L10 11.414l-4.293 4.293a1 1 0 01-1.414-1.414L8.586 10 4.293 5.707a1 1 0 010-1.414z" clipRule="evenodd" />
              </svg>
            </button>
          </div>

          {/* Messages Container */}
          <div className="chatbot-messages-container">
            {state.messages.map((message) => (
              <div
                key={message.id}
                className={`chatbot-message-wrapper ${message.role === 'user' ? 'chatbot-user-message-wrapper' : 'chatbot-assistant-message-wrapper'}`}
              >
                <div
                  className={`chatbot-message ${message.role === 'user' ? 'chatbot-user-message' : 'chatbot-assistant-message'}`}
                >
                  <div className="chatbot-message-content">{message.content}</div>
                  {message.citations && formatCitations(message.citations)}
                  <div className="chatbot-message-timestamp">
                    {format(message.timestamp, 'HH:mm')}
                  </div>
                </div>
              </div>
            ))}
            {state.isTyping && (
              <div className="chatbot-typing-indicator-wrapper">
                <div className="chatbot-typing-indicator">
                  <div className="chatbot-typing-dots">
                    <div className="chatbot-typing-dot"></div>
                    <div className="chatbot-typing-dot"></div>
                    <div className="chatbot-typing-dot"></div>
                  </div>
                  <span className="chatbot-typing-text">Thinking...</span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input Area */}
          <div className="chatbot-input-container">
            <form onSubmit={handleSendMessage} className="chatbot-input-form">
              <input
                type="text"
                value={state.inputMessage}
                onChange={(e) => dispatch({ type: 'SET_INPUT_MESSAGE', payload: e.target.value })}
                placeholder="Ask about robotics..."
                className="chatbot-input"
                disabled={state.isLoading || !state.conversationId}
                onKeyPress={(e) => {
                  if (e.key === 'Enter' && !e.shiftKey) {
                    e.preventDefault();
                    handleSendMessage(e);
                  }
                }}
              />
              <button
                type="submit"
                disabled={state.isLoading || !state.inputMessage.trim() || !state.conversationId}
                className="chatbot-send-button"
              >
                <svg xmlns="http://www.w3.org/2000/svg" className="chatbot-send-icon" viewBox="0 0 20 20" fill="currentColor">
                  <path d="M10.894 2.553a1 1 0 00-1.788 0l-7 14a1 1 0 001.169 1.409l5-1.429A1 1 0 009 15.571V11a1 1 0 112 0v4.571a1 1 0 00.725.962l5 1.428a1 1 0 001.17-1.408l-7-14z" />
                </svg>
              </button>
            </form>
            <div className="chatbot-disclaimer">
              Based on book content
            </div>
          </div>
        </div>
      )}
    </>
  );
};

export default FloatingChatWidget;