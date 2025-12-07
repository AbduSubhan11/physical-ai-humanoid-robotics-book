import React, { useState, useEffect } from "react";
import styles from "./index.module.css";
import aboutStyles from "./about-section.module.css"; // <-- add this


export default function Home() {
  const [scrollY, setScrollY] = useState(0);
  const [activeFeature, setActiveFeature] = useState(0);
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    setIsVisible(true);
    const handleScroll = () => setScrollY(window.scrollY);
    window.addEventListener("scroll", handleScroll);
    return () => window.removeEventListener("scroll", handleScroll);
  }, []);

  useEffect(() => {
    const interval = setInterval(() => {
      setActiveFeature((prev) => (prev + 1) % 4);
    }, 3500);
    return () => clearInterval(interval);
  }, []);

  const features = [
    { icon: "📚", title: "Comprehensive Coverage", description: "Complete Physical AI & Humanoid Robotics curriculum with in-depth chapters and real-world applications", color: "cyan" },
    { icon: "💬", title: "AI-Powered RAG Chatbot", description: "Interactive learning assistant that answers questions using the textbook's knowledge base", color: "violet" },
    { icon: "✨", title: "Personalized Learning", description: "Adaptive content and learning paths tailored to your progress and understanding", color: "pink" },
    { icon: "🌐", title: "Urdu Translation", description: "Full bilingual support making advanced robotics education accessible to Urdu speakers", color: "emerald" },
  ];

  const stats = [
    { icon: "📖", value: "12+", label: "Chapters", color: "cyan" },
    { icon: "👥", value: "1000+", label: "Students", color: "violet" },
    { icon: "🏆", value: "100%", label: "Free Access", color: "emerald" },
  ];

  const aboutFeatures = [
    {
      icon: (
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
          <circle cx="12" cy="12" r="10" />
          <path d="M12 16v-4M12 8h.01" />
        </svg>
      ),
      title: "Deep Understanding",
      description: "Master the fundamental concepts of generative AI and how they apply to physical systems.",
    },
    {
      icon: (
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
          <rect x="4" y="4" width="6" height="6" rx="1" />
          <rect x="14" y="4" width="6" height="6" rx="1" />
          <rect x="4" y="14" width="6" height="6" rx="1" />
          <rect x="14" y="14" width="6" height="6" rx="1" />
        </svg>
      ),
      title: "Hands-On Projects",
      description: "Build and deploy real robotic applications using state-of-the-art AI techniques.",
    },
    {
      icon: (
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
          <polygon points="13 2 3 14 12 14 11 22 21 10 12 10 13 2" />
        </svg>
      ),
      title: "Industry Ready",
      description: "Learn deployment strategies and scaling techniques for production systems.",
    },
  ];

  return (
    <div className={styles.home}>
      {/* Animated Background */}
      <div className={styles.background}>
        <div
          className={styles.mesh}
          style={{
            transform: `translateY(${scrollY * 0.3}px)`,
          }}
        />
      </div>

      {/* Hero Section */}
      <header className={styles.hero}>
        <div className={`${styles.heroContent} ${isVisible ? styles.visible : styles.hidden}`}>
          <div className={styles.badge}>NEXT-GEN AI TEXTBOOK</div>
          <h1 className={styles.title}>
            Physical AI <span className={styles.highlight}>Humanoid Robotics</span>
          </h1>
          <p className={styles.subtitle}>
            Master cutting-edge robotics with an AI-native textbook featuring interactive learning, intelligent assistance, and comprehensive real-world applications
          </p>

          {/* CTA Buttons */}
          <div className={styles.actions}>
            <a href="/docs/intro" className={styles.primaryBtn}>Start Learning</a>
            <a href="#features" className={styles.secondaryBtn}>Explore Features</a>
          </div>

          {/* Stats */}
          <div className={styles.stats}>
            {stats.map((stat, i) => (
              <div key={i} className={styles.statCard}>
                <div className={styles.statIcon}>{stat.icon}</div>
                <div className={`${styles.statValue} ${styles[stat.color]}`}>{stat.value}</div>
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
            <h2>Why Choose This?</h2>
            <p>Revolutionary features for modern learners</p>
          </div>

          <div className={styles.featuresGrid}>
            {features.map((feature, index) => {
              const isActive = activeFeature === index;
              return (
                <div key={index} className={`${styles.featureCard} ${isActive ? styles.active : ''}`} onMouseEnter={() => setActiveFeature(index)}>
                  <div className={styles.featureIcon}>{feature.icon}</div>
                  <h3 className={styles.featureTitle}>{feature.title}</h3>
                  <p className={styles.featureDesc}>{feature.description}</p>
                  {isActive && <div className={`${styles.activeIndicator} ${styles[feature.color]}`} />}
                </div>
              );
            })}
          </div>
        </div>
      </section>

      {/* Final CTA Section */}
      <section className={styles.finalCTA}>
        <div className={styles.ctaContainer}>
          <h2>Ready to Transform Your Learning?</h2>
          <p>Join thousands mastering Physical AI and Humanoid Robotics</p>
          <a href="/docs/intro" className={styles.finalCTAButton}>
            Begin Your Journey
          </a>
        </div>
      </section>

       <section id="about" className={aboutStyles.about}>
        <div className={aboutStyles.container}>
          <div className={aboutStyles.content}>
            <span className={aboutStyles.label}>About the Course</span>
            <h2 className={aboutStyles.title}>Bridge the gap between AI theory and robotics practice</h2>
            <p className={aboutStyles.description}>
              The Physical AI Course is your comprehensive guide to mastering generative AI and humanoid robotics. Dive
              deep into the foundations of physical AI, explore cutting-edge simulation techniques, and build real-world
              robotic applications.
            </p>
            <p className={aboutStyles.subdesc}>
              Designed for engineers, researchers, and enthusiasts who want to transform theoretical knowledge into
              practical, deployable robotics solutions.
            </p>
          </div>

          <div className={aboutStyles.features}>
            {aboutFeatures.map((feature, index) => (
              <div key={index} className={aboutStyles.featureCard}>
                <div className={aboutStyles.featureIcon}>{feature.icon}</div>
                <div className={aboutStyles.featureContent}>
                  <h3 className={aboutStyles.featureTitle}>{feature.title}</h3>
                  <p className={aboutStyles.featureDesc}>{feature.description}</p>
                </div>
              </div>
            ))}
          </div>
        </div>
      </section>
    </div>
  );
}
