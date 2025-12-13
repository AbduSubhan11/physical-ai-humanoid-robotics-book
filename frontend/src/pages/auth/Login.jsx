import React, { useState } from 'react';
import Link from '@docusaurus/Link';

const Login = () => {
  const [formData, setFormData] = useState({
    email: '',
    password: ''
  });
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  const handleChange = (e) => {
    setFormData({
      ...formData,
      [e.target.name]: e.target.value
    });
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError(''); // Reset error
    setLoading(true);

    try {
      // Simulate API call delay
      await new Promise(resolve => setTimeout(resolve, 1000));

      // Simulate a network error for demonstration
      throw new Error('Failed to fetch');
    } catch (error) {
      console.error('Login error:', error);
      setError('Failed to fetch');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="auth-page-container">
      <div className="auth-form-wrapper">
        <div className="auth-header">
          <h2 className="auth-title">
            Login to your account
          </h2>
          <p className="auth-subtitle">
            Or{' '}
            <Link to="/auth/signup" className="auth-link">
              create a new account
            </Link>
          </p>
        </div>

        {/* Error message display */}
        {error && (
          <p className="auth-error-message" style={{ color: 'red', marginBottom: '12px', textAlign: 'center' }}>
            {error}
          </p>
        )}

        <form className="auth-form" onSubmit={handleSubmit}>
          <div className="auth-input-group">
            <div className="auth-input-wrapper">
              <label htmlFor="email" className="auth-input-label sr-only">
                Email address
              </label>
              <input
                id="email"
                name="email"
                type="email"
                autoComplete="email"
                required
                className="auth-input auth-input-top"
                placeholder="Email address"
                value={formData.email}
                onChange={handleChange}
              />
            </div>
            <div className="auth-input-wrapper">
              <label htmlFor="password" className="auth-input-label sr-only">
                Password
              </label>
              <input
                id="password"
                name="password"
                type="password"
                autoComplete="current-password"
                required
                className="auth-input auth-input-bottom"
                placeholder="Password"
                value={formData.password}
                onChange={handleChange}
              />
            </div>
          </div>

          <div className="auth-options">
            <div className="auth-checkbox-group">
              <input
                id="remember-me"
                name="remember-me"
                type="checkbox"
                className="auth-checkbox"
              />
              <label htmlFor="remember-me" className="auth-checkbox-label">
                Remember me
              </label>
            </div>

            <div className="auth-forgot-password">
              <a href="#" className="auth-link">
                Forgot your password?
              </a>
            </div>
          </div>

          <div className="auth-button-wrapper">
            <button
              type="submit"
              disabled={loading}
              className="auth-submit-button"
            >
              {loading ? 'Signing in...' : 'Sign in'}
            </button>
          </div>
        </form>
      </div>
    </div>
  );
};

export default Login;