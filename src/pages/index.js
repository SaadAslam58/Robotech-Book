import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Textbook">
      <main>
        <div className={clsx('hero hero--primary', styles.heroBanner)}>
          <div className="container">
            <h1 className="hero__title">Welcome to {siteConfig.title}</h1>
            <p className="hero__subtitle">{siteConfig.tagline}</p>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/Robotech-Book/part1/chapter1-embodied-intelligence-shift">
                Read the Textbook - Get Started
              </Link>
            </div>
          </div>
        </div>
        <div className="container">
          <div className="row">
            <div className="col col--4 padding--lg">
              <h2>🤖 Physical AI</h2>
              <p>
                Explore the intersection of digital AI and the physical world,
                bridging the gap between LLMs/Agents and robotics.
              </p>
            </div>
            <div className="col col--4 padding--lg">
              <h2>🤖 Humanoid Robotics</h2>
              <p>
                Learn to control humanoid robots using ROS 2, Gazebo, and NVIDIA Isaac.
              </p>
            </div>
            <div className="col col--4 padding--lg">
              <h2>📚 Academic & Practical</h2>
              <p>
                A comprehensive textbook designed for students and developers
                transitioning into robotics.
              </p>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
