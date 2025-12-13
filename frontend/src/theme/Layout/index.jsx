import Layout from '@theme-original/Layout';
import ChatInterface from '../../components/ChatInterface';

export default function LayoutWrapper(props) {
  return (
    <Layout {...props}>
      {props.children}
      <ChatInterface />
    </Layout>
  );
}