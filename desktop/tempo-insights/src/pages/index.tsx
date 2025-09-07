import { useEffect } from 'react';
import { useRouter } from 'next/router';
import { Center, Loader } from '@mantine/core';

export default function LandingPage() {
  const router = useRouter();

  useEffect(() => {
    checkAuth();
  }, []);

  const checkAuth = async () => {
    try {
      const response = await fetch('/api/auth/me');
      if (response.ok) {
        router.push('/home');
      } else {
        router.push('/login');
      }
    } catch (error) {
      router.push('/login');
    }
  };

  return (
    <Center style={{ height: '100vh' }}>
      <Loader size="lg" color="accent" />
    </Center>
  );
}
