// /components/AppLayout.tsx
import { AppShell, Burger, Group, Text, NavLink, Stack } from '@mantine/core';
import { useDisclosure } from '@mantine/hooks';
import { useRouter } from 'next/router';
import Link from 'next/link';
import { ReactNode, useEffect, useState } from 'react';
import {
  IconHome,
  IconUser,
  IconUsers,
  IconCpu,
  IconLogout,
  IconMapPin
} from '@tabler/icons-react';
import { logout } from '../lib/auth/logout';

interface AppLayoutProps {
  children: ReactNode;
}

export function AppLayout({ children }: AppLayoutProps) {
  const [opened, { toggle }] = useDisclosure();
  const router = useRouter();
  const [userRole, setUserRole] = useState<string | null>(null);

  useEffect(() => {
    // Get user role from localStorage or API
    const fetchUserRole = async () => {
      try {
        const response = await fetch('/api/auth/me');
        if (response.ok) {
          const data = await response.json();
          setUserRole(data.user.role);
        }
      } catch (error) {
        console.error('Error fetching user role:', error);
      }
    };
    fetchUserRole();
  }, []);

  const handleLogout = async () => {
    await logout();
  };

  const isAdmin = userRole === 'ADMIN' || userRole === 'SUPER_ADMIN';

  return (
    <AppShell
      header={{ height: 60 }}
      navbar={{
        width: 250,
        breakpoint: 'sm',
        collapsed: { mobile: !opened }
      }}
      aside={{
        width: 300,
        breakpoint: 'md',
        collapsed: { desktop: false, mobile: true }
      }}
      padding="md"
    >
      <AppShell.Header>
        <Group h="100%" px="md">
          <Burger
            opened={opened}
            onClick={toggle}
            hiddenFrom="sm"
            size="sm"
          />
          <Text size="xl" fw={700} c="accent.6">Tempo Insights</Text>
        </Group>
      </AppShell.Header>

      <AppShell.Navbar p="md">
        <Stack gap="xs" style={{ flex: 1 }}>
          <NavLink
            component={Link}
            href="/home"
            label="Home"
            leftSection={<IconHome size={16} />}
            active={router.pathname === '/home'}
          />
          <NavLink
            component={Link}
            href="/profile"
            label="Profile"
            leftSection={<IconUser size={16} />}
            active={router.pathname === '/profile'}
          />
          <NavLink
            component={Link}
            href="/groups"
            label="Groups"
            leftSection={<IconUsers size={16} />}
            active={router.pathname === '/groups'}
          />
          {isAdmin && (
            <>
              <NavLink
                component={Link}
                href="/devices"
                label="Devices (Admin)"
                leftSection={<IconCpu size={16} />}
                active={router.pathname === '/devices'}
              />
              <NavLink
                component={Link}
                href="/dropzones"
                label="Dropzones (Admin)"
                leftSection={<IconMapPin size={16} />}
                active={router.pathname === '/dropzones'}
              />
            </>
          )}
          <div style={{ flex: 1 }} />
          <NavLink
            label="Logout"
            leftSection={<IconLogout size={16} />}
            onClick={handleLogout}
          />
        </Stack>
      </AppShell.Navbar>

      <AppShell.Main>
        {children}
      </AppShell.Main>

    </AppShell>
  );
}