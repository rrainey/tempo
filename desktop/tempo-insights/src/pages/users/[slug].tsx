import { useEffect, useState } from 'react';
import { useRouter } from 'next/router';
import {
  Container,
  Title,
  Paper,
  Text,
  Group,
  Avatar,
  Badge,
  Tabs,
  Stack,
  Card
} from '@mantine/core';
import { AppLayout } from '../../components/AppLayout';
import { AuthGuard } from '../../components/AuthGuard';
import { IconUser, IconParachute, IconUsers } from '@tabler/icons-react';

interface UserProfile {
  id: string;
  name: string;
  email: string;
  slug: string;
  role: string;
  createdAt: string;
  jumpCount?: number;
  groupCount?: number;
}

export default function UserProfilePage() {
  const router = useRouter();
  const { slug } = router.query;
  const [user, setUser] = useState<UserProfile | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (slug) {
      loadUserProfile();
    }
  }, [slug]);

  const loadUserProfile = async () => {
    try {
      // TODO: Implement user profile API endpoint
      // const response = await fetch(`/api/users/${slug}`);
      // const data = await response.json();
      // setUser(data.user);

      // For now, load a mock profile for admin
      if (slug === 'admin') {
        setUser({
          id: 'admin-id',
          name: 'Admin User',
          email: 'admin@tempoinsights.local',
          slug: 'admin',
          role: 'SUPER_ADMIN',
          createdAt: new Date().toISOString(),
          jumpCount: 0,
          groupCount: 0,
        });
      } else {
        setError('User not found');
      }
      setLoading(false);
    } catch (err) {
      setError('Failed to load user profile');
      setLoading(false);
    }
  };

  const getRoleBadgeColor = (role: string) => {
    switch (role) {
      case 'SUPER_ADMIN': return 'red';
      case 'ADMIN': return 'orange';
      default: return 'blue';
    }
  };

  if (loading) return (
    <AuthGuard>
      <AppLayout>
        <Container>
          <Text>Loading user profile...</Text>
        </Container>
      </AppLayout>
    </AuthGuard>
  );

  if (error || !user) return (
    <AuthGuard>
      <AppLayout>
        <Container>
          <Text c="red">{error || 'User not found'}</Text>
        </Container>
      </AppLayout>
    </AuthGuard>
  );

  return (
    <AuthGuard>
      <AppLayout>
        <Container>
          <Paper p="xl" withBorder mb="xl">
            <Group>
              <Avatar size="xl" radius="md" color="accent">
                {user.name.split(' ').map(n => n[0]).join('').toUpperCase()}
              </Avatar>
              <div style={{ flex: 1 }}>
                <Group justify="space-between">
                  <div>
                    <Title order={2}>{user.name}</Title>
                    <Text c="dimmed">@{user.slug}</Text>
                  </div>
                  <Badge color={getRoleBadgeColor(user.role)}>
                    {user.role.replace('_', ' ')}
                  </Badge>
                </Group>
                <Group mt="xs">
                  <Text size="sm" c="dimmed">
                    Member since {new Date(user.createdAt).toLocaleDateString()}
                  </Text>
                </Group>
              </div>
            </Group>
          </Paper>

          <Tabs defaultValue="jumps">
            <Tabs.List>
              <Tabs.Tab value="jumps" leftSection={<IconParachute size={16} />}>
                Jumps ({user.jumpCount || 0})
              </Tabs.Tab>
              <Tabs.Tab value="groups" leftSection={<IconUsers size={16} />}>
                Groups ({user.groupCount || 0})
              </Tabs.Tab>
              <Tabs.Tab value="devices" leftSection={<IconUser size={16} />}>
                Devices
              </Tabs.Tab>
            </Tabs.List>

            <Tabs.Panel value="jumps" pt="md">
              <Card p="xl" withBorder>
                <Text c="dimmed" ta="center">No jumps recorded yet</Text>
              </Card>
            </Tabs.Panel>

            <Tabs.Panel value="groups" pt="md">
              <Card p="xl" withBorder>
                <Text c="dimmed" ta="center">Not a member of any groups</Text>
              </Card>
            </Tabs.Panel>

            <Tabs.Panel value="devices" pt="md">
              <Card p="xl" withBorder>
                <Text c="dimmed" ta="center">No devices registered</Text>
              </Card>
            </Tabs.Panel>
          </Tabs>
        </Container>
      </AppLayout>
    </AuthGuard>
  );
}
