import React, { useState } from 'react';
import Link from '@docusaurus/Link';

const Signup = () => {
  const [formData, setFormData] = useState({
    name: '',
    email: '',
    password: '',
    confirmPassword: ''
  });
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(''); // ✅ added

  const handleChange = (e) => {
    setFormData({
      ...formData,
      [e.target.name]: e.target.value
    });
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError(''); // ✅ reset error

    // Basic validation
    if (formData.password !== formData.confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    if (formData.password.length < 6) {
      setError('Password must be at least 6 characters long');
      return;
    }

    setLoading(true);

    try {
      // Simulate API call delay
      await new Promise(resolve => setTimeout(resolve, 1000));

      // Simulate a network error for demonstration
      throw new Error('Failed to fetch');
    } catch (error) {
      console.error('Signup error:', error);
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
            Create a new account
          </h2>
          <p className="auth-subtitle">
            Or{' '}
            <Link to="/auth/login" className="auth-link">
              Login to your account
            </Link>
          </p>
        </div>

        {/* ✅ Inline error message */}
        {error && (
          <p style={{ color: 'red', marginBottom: '12px', textAlign: 'center' }}>
            {error}
          </p>
        )}

        <form className="auth-form" onSubmit={handleSubmit}>
          <div className="auth-input-group">
            <div className="auth-input-wrapper">
              <label htmlFor="name" className="auth-input-label sr-only">
                Full Name
              </label>
              <input
                id="name"
                name="name"
                type="text"
                autoComplete="name"
                required
                className="auth-input auth-input-top"
                placeholder="Full Name"
                value={formData.name}
                onChange={handleChange}
              />
            </div>

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
                className="auth-input"
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
                autoComplete="new-password"
                required
                className="auth-input"
                placeholder="Password"
                value={formData.password}
                onChange={handleChange}
              />
            </div>

            <div className="auth-input-wrapper">
              <label htmlFor="confirmPassword" className="auth-input-label sr-only">
                Confirm Password
              </label>
              <input
                id="confirmPassword"
                name="confirmPassword"
                type="password"
                autoComplete="new-password"
                required
                className="auth-input auth-input-bottom"
                placeholder="Confirm Password"
                value={formData.confirmPassword}
                onChange={handleChange}
              />
            </div>
          </div>

          <div className="auth-terms-group">
            <input
              id="terms"
              name="terms"
              type="checkbox"
              required
              className="auth-checkbox"
            />
            <label htmlFor="terms" className="auth-checkbox-label">
              I agree to the Terms and Conditions{' '}
            </label>
          </div>

          <div className="auth-button-wrapper">
            <button
              type="submit"
              disabled={loading}
              className="auth-submit-button"
            >
              {loading ? 'Creating account...' : 'Sign up'}
            </button>
          </div>
        </form>
      </div>
    </div>
  );
};

export default Signup;
