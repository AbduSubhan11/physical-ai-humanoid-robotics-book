import React, { useState, useEffect } from "react";
import styles from "./index.module.css";

export default function Home() {
  const [scrollY, setScrollY] = useState(0);
  const [activeFeature, setActiveFeature] = useState(0);

  useEffect(() => {
    const handleScroll = () => setScrollY(window.scrollY);
    window.addEventListener("scroll", handleScroll);
    return () => window.removeEventListener("scroll", handleScroll);
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
    <div className={styles.home}>
      {/* Animated Background */}
      <div className={styles.glowBlue} style={{ transform: `translateY(${scrollY * 0.3}px)` }}></div>
      <div className={styles.glowPurple} style={{ transform: `translateY(${-scrollY * 0.2}px)` }}></div>

      {/* Hero Section */}
      <header className={styles.hero}>
        <div className={styles.heroContent}>
          <div className={styles.badge}>⚡ AI-Native Textbook</div>
          <h1 className={styles.title}>
            Physical AI & <br /> Humanoid Robotics
          </h1>
          <p className={styles.subtitle}>
            Master the future of robotics with interactive learning, AI-powered assistance, and comprehensive coverage of cutting-edge technologies
          </p>

          <div className={styles.actions}>
            <a href="/docs/intro" className={styles.primaryBtn}>
              Start Reading ➡️
            </a>
            <a href="#features" className={styles.secondaryBtn}>
              Explore Features
            </a>
          </div>

          <div className={styles.stats}>
            {stats.map((stat, i) => (
              <div key={i} className={styles.stat}>
                <div className={styles.statIcon}>{stat.icon}</div>
                <div className={styles.statValue}>{stat.value}</div>
                <div className={styles.statLabel}>{stat.label}</div>
              </div>
            ))}
          </div>
        </div>
      </header>

      {/* Features Section */}
      <section id="features" className={styles.featuresSection}>
        <div className={styles.featuresContainer}>
          <div className={styles.featuresHeader}>
            <h2>Why This Textbook?</h2>
            <p>Experience the future of technical education</p>
          </div>

          <div className={styles.featuresGrid}>
            {features.map((feature, index) => (
              <div
                key={index}
                className={`${styles.featureCard} ${activeFeature === index ? styles.active : ""}`}
                onMouseEnter={() => setActiveFeature(index)}
              >
                <div className={styles.featureIcon}>{feature.icon}</div>
                <h3 className={styles.featureTitle}>{feature.title}</h3>
                <p className={styles.featureDesc}>{feature.description}</p>
              </div>
            ))}
          </div>
        </div>
      </section>
    </div>
  );
}
