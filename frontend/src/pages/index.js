import React from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={'hero shadow--lw--darkest round-border'+styles.heroBanner}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        {/* Add your homepage content here */}
        <section className={styles.features}>
            <div className="container">
                <div className="row">
                    <div className="col col--6">
                        <h2>Welcome to the AI-Native Textbook!</h2>
                        <p>This textbook teaches the entire Physical AI & Humanoid Robotics course.</p>
                    </div>
                    <div className="col col--6">
                        <h2>Features</h2>
                        <ul>
                            <li>Docusaurus-powered technical book</li>
                            <li>Integrated RAG Chatbot</li>
                            <li>Personalization</li>
                            <li>Urdu Translation</li>
                        </ul>
                    </div>
                </div>
            </div>
        </section>
      </main>
    </Layout>
  );
}
