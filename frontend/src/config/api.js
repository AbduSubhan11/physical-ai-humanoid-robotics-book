// API configuration
// Using window object to access environment variables in Docusaurus
const API_BASE_URL =
  typeof window !== 'undefined' && window.REACT_APP_API_URL
    ? window.REACT_APP_API_URL
    : 'http://localhost:8000/api/v1';

const WS_BASE_URL =
  typeof window !== 'undefined' && window.REACT_APP_WS_URL
    ? window.REACT_APP_WS_URL
    : 'ws://localhost:8000';

export { API_BASE_URL, WS_BASE_URL };