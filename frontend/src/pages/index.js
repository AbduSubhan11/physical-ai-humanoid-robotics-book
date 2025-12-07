import React, { useState, useEffect } from 'react';
import { Book, Brain, Cpu, Zap, Globe, MessageSquare, Sparkles, ChevronRight, BookOpen, Users, Award } from 'lucide-react';

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
    {
      icon: <Book className="w-8 h-8" />,
      title: "Comprehensive Coverage",
      description: "Complete Physical AI & Humanoid Robotics curriculum with in-depth chapters and real-world applications"
    },
    {
      icon: <MessageSquare className="w-8 h-8" />,
      title: "AI-Powered RAG Chatbot",
      description: "Interactive learning assistant that answers questions using the textbook's knowledge base"
    },
    {
      icon: <Sparkles className="w-8 h-8" />,
      title: "Personalized Learning",
      description: "Adaptive content and learning paths tailored to your progress and understanding"
    },
    {
      icon: <Globe className="w-8 h-8" />,
      title: "Urdu Translation",
      description: "Full bilingual support making advanced robotics education accessible to Urdu speakers"
    }
  ];

  const stats = [
    { icon: <BookOpen className="w-6 h-6" />, value: "12+", label: "Chapters" },
    { icon: <Users className="w-6 h-6" />, value: "1000+", label: "Students" },
    { icon: <Award className="w-6 h-6" />, value: "100%", label: "Free Access" }
  ];

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-950 via-blue-950 to-slate-950 text-white overflow-hidden">
      {/* Animated Background Elements */}
      <div className="fixed inset-0 overflow-hidden pointer-events-none">
        <div 
          className="absolute top-1/4 -left-20 w-96 h-96 bg-blue-500/10 rounded-full blur-3xl animate-pulse"
          style={{ transform: `translateY(${scrollY * 0.3}px)` }}
        />
        <div 
          className="absolute bottom-1/4 -right-20 w-96 h-96 bg-purple-500/10 rounded-full blur-3xl animate-pulse"
          style={{ transform: `translateY(${-scrollY * 0.2}px)` }}
        />
      </div>

      {/* Hero Section */}
      <header className="relative min-h-screen flex items-center justify-center px-4 sm:px-6 lg:px-8">
        <div className="max-w-6xl mx-auto text-center">
          {/* Floating Badge */}
          <div className="inline-flex items-center gap-2 px-4 py-2 bg-blue-500/20 backdrop-blur-sm border border-blue-400/30 rounded-full mb-8 animate-fade-in">
            <Zap className="w-4 h-4 text-yellow-400" />
            <span className="text-sm font-medium">AI-Native Textbook</span>
          </div>

          {/* Main Title */}
          <h1 className="text-5xl sm:text-6xl lg:text-7xl font-bold mb-6 bg-gradient-to-r from-blue-400 via-purple-400 to-pink-400 bg-clip-text text-transparent animate-fade-in-up">
            Physical AI &
            <br />
            Humanoid Robotics
          </h1>

          <p className="text-xl sm:text-2xl text-slate-300 mb-12 max-w-3xl mx-auto animate-fade-in-up" style={{ animationDelay: '0.2s' }}>
            Master the future of robotics with interactive learning, AI-powered assistance, and comprehensive coverage of cutting-edge technologies
          </p>

          {/* CTA Buttons */}
          <div className="flex flex-col sm:flex-row gap-4 justify-center items-center animate-fade-in-up" style={{ animationDelay: '0.4s' }}>
            <a 
              href="/docs/intro" 
              className="group px-8 py-4 bg-gradient-to-r from-blue-600 to-purple-600 rounded-xl font-semibold text-lg flex items-center gap-2 hover:scale-105 transition-transform shadow-lg shadow-blue-500/50"
            >
              Start Reading
              <ChevronRight className="w-5 h-5 group-hover:translate-x-1 transition-transform" />
            </a>
            <a 
              href="#features" 
              className="px-8 py-4 bg-white/10 backdrop-blur-sm border border-white/20 rounded-xl font-semibold text-lg hover:bg-white/20 transition-all"
            >
              Explore Features
            </a>
          </div>

          {/* Stats */}
          <div className="grid grid-cols-3 gap-8 mt-20 max-w-2xl mx-auto animate-fade-in-up" style={{ animationDelay: '0.6s' }}>
            {stats.map((stat, index) => (
              <div key={index} className="text-center">
                <div className="flex justify-center mb-2 text-blue-400">
                  {stat.icon}
                </div>
                <div className="text-3xl font-bold mb-1">{stat.value}</div>
                <div className="text-sm text-slate-400">{stat.label}</div>
              </div>
            ))}
          </div>
        </div>

        {/* Scroll Indicator */}
        <div className="absolute bottom-8 left-1/2 -translate-x-1/2 animate-bounce">
          <div className="w-6 h-10 border-2 border-white/30 rounded-full flex justify-center pt-2">
            <div className="w-1 h-3 bg-white rounded-full" />
          </div>
        </div>
      </header>

      {/* Features Section */}
      <section id="features" className="relative py-20 px-4 sm:px-6 lg:px-8">
        <div className="max-w-6xl mx-auto">
          <div className="text-center mb-16">
            <h2 className="text-4xl sm:text-5xl font-bold mb-4">
              Why This Textbook?
            </h2>
            <p className="text-xl text-slate-300">
              Experience the future of technical education
            </p>
          </div>

          <div className="grid md:grid-cols-2 gap-8">
            {features.map((feature, index) => (
              <div
                key={index}
                className={`group p-8 rounded-2xl backdrop-blur-sm border transition-all duration-500 hover:scale-105 cursor-pointer ${
                  activeFeature === index
                    ? 'bg-gradient-to-br from-blue-600/20 to-purple-600/20 border-blue-400/50 shadow-xl shadow-blue-500/20'
                    : 'bg-white/5 border-white/10 hover:bg-white/10'
                }`}
                onMouseEnter={() => setActiveFeature(index)}
              >
                <div className={`inline-flex p-3 rounded-xl mb-4 transition-colors ${
                  activeFeature === index ? 'bg-blue-500/20 text-blue-400' : 'bg-white/10 text-white'
                }`}>
                  {feature.icon}
                </div>
                <h3 className="text-2xl font-bold mb-3">{feature.title}</h3>
                <p className="text-slate-300 leading-relaxed">{feature.description}</p>
              </div>
            ))}
          </div>
        </div>
      </section>

      {/* Tech Stack Section */}
      <section className="relative py-20 px-4 sm:px-6 lg:px-8 bg-gradient-to-b from-transparent to-slate-900/50">
        <div className="max-w-6xl mx-auto text-center">
          <h2 className="text-4xl sm:text-5xl font-bold mb-12">
            Built with Modern Technology
          </h2>
          <div className="flex flex-wrap justify-center gap-6">
            {[
              { icon: <Cpu className="w-8 h-8" />, name: "Docusaurus" },
              { icon: <Brain className="w-8 h-8" />, name: "RAG AI" },
              { icon: <Zap className="w-8 h-8" />, name: "Real-time Updates" },
              { icon: <Globe className="w-8 h-8" />, name: "Multilingual" }
            ].map((tech, index) => (
              <div
                key={index}
                className="flex items-center gap-3 px-6 py-4 bg-white/5 backdrop-blur-sm border border-white/10 rounded-xl hover:bg-white/10 hover:border-white/20 transition-all"
              >
                <div className="text-blue-400">{tech.icon}</div>
                <span className="font-semibold">{tech.name}</span>
              </div>
            ))}
          </div>
        </div>
      </section>

      {/* CTA Section */}
      <section className="relative py-20 px-4 sm:px-6 lg:px-8">
        <div className="max-w-4xl mx-auto text-center">
          <div className="p-12 rounded-3xl bg-gradient-to-r from-blue-600/20 to-purple-600/20 backdrop-blur-sm border border-blue-400/30">
            <h2 className="text-4xl font-bold mb-4">
              Ready to Start Learning?
            </h2>
            <p className="text-xl text-slate-300 mb-8">
              Join thousands of students mastering Physical AI and Humanoid Robotics
            </p>
            <a
              href="/docs/intro"
              className="inline-flex items-center gap-2 px-10 py-5 bg-gradient-to-r from-blue-600 to-purple-600 rounded-xl font-bold text-lg hover:scale-105 transition-transform shadow-xl shadow-blue-500/50"
            >
              Begin Your Journey
              <ChevronRight className="w-5 h-5" />
            </a>
          </div>
        </div>
      </section>

      {/* Footer */}
      <footer className="relative py-12 px-4 sm:px-6 lg:px-8 border-t border-white/10">
        <div className="max-w-6xl mx-auto text-center text-slate-400">
          <p>Copyright © {new Date().getFullYear()} Physical AI & Humanoid Robotics. Built with Docusaurus.</p>
          <div className="flex justify-center gap-6 mt-4">
            <a href="https://discord.gg/your-discord-link" className="hover:text-blue-400 transition-colors">Discord</a>
            <a href="https://twitter.com/your-twitter-handle" className="hover:text-blue-400 transition-colors">Twitter</a>
            <a href="https://github.com/abdusubhan11/physical-ai-humanoid-robotics-book" className="hover:text-blue-400 transition-colors">GitHub</a>
          </div>
        </div>
      </footer>

      <style jsx>{`
        @keyframes fade-in {
          from { opacity: 0; }
          to { opacity: 1; }
        }
        
        @keyframes fade-in-up {
          from {
            opacity: 0;
            transform: translateY(30px);
          }
          to {
            opacity: 1;
            transform: translateY(0);
          }
        }
        
        .animate-fade-in {
          animation: fade-in 0.8s ease-out forwards;
        }
        
        .animate-fade-in-up {
          animation: fade-in-up 0.8s ease-out forwards;
          opacity: 0;
        }
      `}</style>
    </div>
  );
}