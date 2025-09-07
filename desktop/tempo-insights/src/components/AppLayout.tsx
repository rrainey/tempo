import { AppShell, Burger, Group, Text } from '@mantine/core';
import { useDisclosure } from '@mantine/hooks';
import { ReactNode } from 'react';

interface AppLayoutProps {
  children: ReactNode;
}

export function AppLayout({ children }: AppLayoutProps) {
  const [opened, { toggle }] = useDisclosure();

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
        <Text>Left Navigation</Text>
        {/* Navigation items will go here */}
      </AppShell.Navbar>

      <AppShell.Main>
        {children}
      </AppShell.Main>

      <AppShell.Aside p="md">
        <Text>Right Sidebar</Text>
        {/* Activity feed will go here */}
      </AppShell.Aside>
    </AppShell>
  );
}
