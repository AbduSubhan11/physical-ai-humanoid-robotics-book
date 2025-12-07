import React, { useState, useEffect } from 'react';
import './index.module.css'; // your CSS file

export default function Home() {
  const [scrollY, setScrollY] = useState(0);
  const [activeFeature, setActiveFeature] = useState(0);

  useEffect(() => {
    const handleScroll = () => setScrollY(window.scrollY);
    window.addEventListener('scroll', handleScroll);
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  useEffect(() => {
    const interval = setInterval(() => {
      setActiveFeature((prev) => (prev + 1) % 4);
    }, 3000);
    return () => clearInterval(interval);
  }, []);

  const features = [
    { icon: "📚", title: "Comprehensive Coverage", description: "Complete Physical AI & Humanoid Robotics curriculum with in-depth chapters and real-world applications" },
    { icon: "💬", title: "AI-Powered RAG Chatbot", description: "Interactive learning assistant that answers questions using the textbook's knowledge base" },
    { icon: "✨", title: "Personalized Learning", description: "Adaptive content and learning paths tailored to your progress and understanding" },
    { icon: "🌐", title: "Urdu Translation", description: "Full bilingual support making advanced robotics education accessible to Urdu speakers" },
  ];

  const stats = [
    { icon: "📖", value: "12+", label: "Chapters" },
    { icon: "👥", value: "1000+", label: "Students" },
    { icon: "🏆", value: "100%", label: "Free Access" },
  ];

  return (
    <div className="home-container">
      {/* Background Elements */}
      <div className="background-circle blue" style={{ transform: `translateY(${scrollY * 0.3}px)` }}></div>
      <div className="background-circle purple" style={{ transform: `translateY(${-scrollY * 0.2}px)` }}></div>

      {/* Hero Section */}
      <header className="hero-section">
        <div className="hero-content">
          <div className="badge">⚡ AI-Native Textbook</div>

          <h1 className="hero-title">
            Physical AI & <br /> Humanoid Robotics
          </h1>

          <p className="hero-subtitle">
            Master the future of robotics with interactive learning, AI-powered assistance, and comprehensive coverage of cutting-edge technologies
          </p>

          <div className="hero-buttons">
            <a href="/docs/intro" className="btn-primary">
              Start Reading ➡️
            </a>
            <a href="#features" className="btn-secondary">
              Explore Features
            </a>
          </div>

          {/* Stats */}
          <div className="stats-container">
            {stats.map((stat, index) => (
              <div key={index} className="stat">
                <div className="stat-icon">{stat.icon}</div>
                <div className="stat-value">{stat.value}</div>
                <div className="stat-label">{stat.label}</div>
              </div>
            ))}
          </div>
        </div>
      </header>

      {/* Features Section */}
      <section id="features" className="features-section">
        <div className="features-container">
          <div className="features-header">
            <h2>Why This Textbook?</h2>
            <p>Experience the future of technical education</p>
          </div>

          <div className="features-grid">
            {features.map((feature, index) => (
              <div
                key={index}
                className={`feature-card ${activeFeature === index ? 'active' : ''}`}
                onMouseEnter={() => setActiveFeature(index)}
              >
                <div className="feature-icon">{feature.icon}</div>
                <h3 className="feature-title">{feature.title}</h3>
                <p className="feature-desc">{feature.description}</p>
              </div>
            ))}
          </div>
        </div>
      </section>
    </div>
  );
}
