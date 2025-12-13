import axios from 'axios';

// Create an axios instance with base configuration
const apiClient = axios.create({
  baseURL: 'http://localhost:8000/api/v1', // Update this to your backend URL
  timeout: 30000, // 30 seconds timeout
  headers: {
    'Content-Type': 'application/json',
  },
});

// Request interceptor to add auth token if available
apiClient.interceptors.request.use(
  (config) => {
    const token = localStorage.getItem('token');
    if (token) {
      config.headers.Authorization = `Bearer ${token}`;
    }
    return config;
  },
  (error) => {
    return Promise.reject(error);
  }
);

// Response interceptor to handle errors globally
apiClient.interceptors.response.use(
  (response) => {
    return response;
  },
  (error) => {
    // Handle specific error cases
    if (error.response?.status === 401) {
      // Token might be expired, redirect to login
      localStorage.removeItem('token');
      // Optionally redirect to login page
    }

    return Promise.reject(error);
  }
);

// Content API functions
export const contentAPI = {
  upload: async (file) => {
    const formData = new FormData();
    formData.append('file', file);

    return apiClient.post('/content/upload', formData, {
      headers: {
        'Content-Type': 'multipart/form-data',
      },
    });
  },

  getProcessingStatus: async (jobId) => {
    return apiClient.get(`/content/processing-status/${jobId}`);
  },
};

// Chat API functions
export const chatAPI = {
  startConversation: async () => {
    return apiClient.post('/chat/start');
  },

  sendMessage: async (conversationId, message) => {
    return apiClient.post(`/chat/${conversationId}/message`, {
      message,
    });
  },

  getHistory: async (conversationId) => {
    return apiClient.get(`/chat/${conversationId}/history`);
  },
};

// Search API functions
export const searchAPI = {
  semanticSearch: async (query, topK = 5) => {
    return apiClient.post('/search', {
      query,
      top_k: topK,
    });
  },
};

// Admin API functions
export const adminAPI = {
  getContentList: async () => {
    return apiClient.get('/admin/content');
  },

  deleteContent: async (contentId) => {
    return apiClient.delete(`/admin/content/${contentId}`);
  },
};

// Authentication API functions
export const authAPI = {
  register: async (userData) => {
    return apiClient.post('/auth/register', userData);
  },

  login: async (credentials) => {
    return apiClient.post('/auth/login', credentials);
  },
};

export default apiClient;