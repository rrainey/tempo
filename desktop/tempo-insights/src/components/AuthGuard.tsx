// /components/AuthGuard.tsx (updated to add requireAdmin)
import { useEffect, useState } from 'react';
import { useRouter } from 'next/router';
import { Center, Loader } from '@mantine/core';

interface AuthGuardProps {
  children: React.ReactNode;
  requireAdmin?: boolean;
}

export function AuthGuard({ children, requireAdmin = false }: AuthGuardProps) {
  const router = useRouter();
  const [loading, setLoading] = useState(true);
  const [authenticated, setAuthenticated] = useState(false);

  useEffect(() => {
    checkAuth();
  }, []);

  const checkAuth = async () => {
    try {
      const response = await fetch('/api/auth/me');
      if (response.ok) {
        if (requireAdmin) {
          const data = await response.json();
          const userRole = data.user.role;
          if (userRole !== 'ADMIN' && userRole !== 'SUPER_ADMIN') {
            router.push('/home');
            return;
          }
        }
        setAuthenticated(true);
      } else {
        router.push('/login');
      }
    } catch (error) {
      router.push('/login');
    } finally {
      setLoading(false);
    }
  };

  if (loading) {
    return (
      <Center style={{ height: '100vh' }}>
        <Loader size="lg" color="accent" />
      </Center>
    );
  }

  if (!authenticated) {
    return null;
  }

  return <>{children}</>;
}