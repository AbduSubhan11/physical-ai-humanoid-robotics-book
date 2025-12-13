import React, { createContext, useContext, useReducer } from 'react';

const ChatContext = createContext();

const chatReducer = (state, action) => {
  switch (action.type) {
    case 'SET_IS_OPEN':
      return { ...state, isOpen: action.payload };
    case 'SET_MESSAGES':
      return { ...state, messages: action.payload };
    case 'ADD_MESSAGE':
      return { ...state, messages: [...state.messages, action.payload] };
    case 'SET_LOADING':
      return { ...state, isLoading: action.payload };
    case 'SET_TYPING':
      return { ...state, isTyping: action.payload };
    case 'SET_CONVERSATION_ID':
      return { ...state, conversationId: action.payload };
    case 'SET_INPUT_MESSAGE':
      return { ...state, inputMessage: action.payload };
    case 'CLEAR_CHAT':
      return {
        ...state,
        
        messages: [
          {
            id: 'welcome',
            role: 'assistant',
            content: 'Hello! I\'m your AI assistant for the Physical AI & Humanoid Robotics book. Ask me anything about robotics concepts, and I\'ll provide answers based on the book content.',
            timestamp: new Date()
          }
        ],
        conversationId: null
      };
    default:
      return state;
  }
};

const ChatProvider = ({ children }) => {
  const [state, dispatch] = useReducer(chatReducer, {
    isOpen: false,
    messages: [
      {
        id: 'welcome',
        role: 'assistant',
        content: 'Hello! I\'m your AI assistant for the Physical AI & Humanoid Robotics book. Ask me anything about robotics concepts, and I\'ll provide answers based on the book content.',
        timestamp: new Date()
      }
    ],
    inputMessage: '',
    isLoading: false,
    isTyping: false,
    conversationId: null
  });

  const value = {
    state,
    dispatch
  };

  return (
    <ChatContext.Provider value={value}>
      {children}
    </ChatContext.Provider>
  );
};

const useChat = () => {
  const context = useContext(ChatContext);
  if (!context) {
    throw new Error('useChat must be used within a ChatProvider');
  }
  return context;
};

export { ChatProvider, useChat };