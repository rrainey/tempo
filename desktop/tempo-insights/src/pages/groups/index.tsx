import { useState, useEffect } from 'react';
import { Container, Title, Text, Button, Group as MantineGroup, Card, Stack, Badge, Grid } from '@mantine/core';
import { AppLayout } from '../../components/AppLayout';
import { AuthGuard } from '../../components/AuthGuard';
import { CreateGroupModal } from '../../components/CreateGroupModal';
import { IconPlus, IconUsers } from '@tabler/icons-react';
import Link from 'next/link';

interface Group {
  id: string;
  name: string;
  slug: string;
  description: string | null;
  isPublic: boolean;
  memberCount: number;
  userRole?: string;
}

export default function GroupsPage() {
  const [createModalOpened, setCreateModalOpened] = useState(false);
  const [myGroups, setMyGroups] = useState<Group[]>([]);
  const [publicGroups, setPublicGroups] = useState<Group[]>([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    loadGroups();
  }, []);

  const loadGroups = async () => {
    try {
      const response = await fetch('/api/groups');
      if (!response.ok) throw new Error('Failed to load groups');

      const data = await response.json();
      setMyGroups(data.myGroups);
      setPublicGroups(data.publicGroups);
    } catch (error) {
      console.error('Error loading groups:', error);
    } finally {
      setLoading(false);
    }
  };

  const getRoleBadgeColor = (role?: string) => {
    switch (role) {
      case 'OWNER': return 'red';
      case 'ADMIN': return 'orange';
      default: return 'gray';
    }
  };

  return (
    <AuthGuard>
      <AppLayout>
        <Container>
          <MantineGroup justify="space-between" mb="xl">
            <Title order={2}>Groups</Title>
            <Button
              leftSection={<IconPlus size={16} />}
              onClick={() => setCreateModalOpened(true)}
            >
              Create Group
            </Button>
          </MantineGroup>

          <Stack gap="xl">
            {/* My Groups */}
            <div>
              <Title order={3} mb="md">My Groups</Title>
              {myGroups.length === 0 ? (
                <Text c="dimmed">You&apos;re not a member of any groups yet.</Text>
              ) : (
                <Grid>
                  {myGroups.map(group => (
                    <Grid.Col key={group.id} span={{ base: 12, sm: 6, md: 4 }}>
                      <Card
                        component={Link}
                        href={`/groups/${group.slug}`}
                        p="lg"
                        withBorder
                        style={{ cursor: 'pointer' }}
                      >
                        <MantineGroup justify="space-between" mb="xs">
                          <Text fw={500}>{group.name}</Text>
                          {group.userRole && (
                            <Badge color={getRoleBadgeColor(group.userRole)} size="sm">
                              {group.userRole}
                            </Badge>
                          )}
                        </MantineGroup>
                        {group.description && (
                          <Text size="sm" c="dimmed" lineClamp={2} mb="xs">
                            {group.description}
                          </Text>
                        )}
                        <MantineGroup gap="xs">
                          <IconUsers size={16} />
                          <Text size="sm">{group.memberCount} members</Text>
                        </MantineGroup>
                      </Card>
                    </Grid.Col>
                  ))}
                </Grid>
              )}
            </div>

            {/* Public Groups */}
            <div>
              <Title order={3} mb="md">Discover Public Groups</Title>
              {publicGroups.length === 0 ? (
                <Text c="dimmed">No public groups available to join.</Text>
              ) : (
                <Grid>
                  {publicGroups.map(group => (
                    <Grid.Col key={group.id} span={{ base: 12, sm: 6, md: 4 }}>
                      <Card
                        component={Link}
                        href={`/groups/${group.slug}`}
                        p="lg"
                        withBorder
                        style={{ cursor: 'pointer' }}
                      >
                        <MantineGroup justify="space-between" mb="xs">
                          <Text fw={500}>{group.name}</Text>
                          <Badge color="green" size="sm">Public</Badge>
                        </MantineGroup>
                        {group.description && (
                          <Text size="sm" c="dimmed" lineClamp={2} mb="xs">
                            {group.description}
                          </Text>
                        )}
                        <MantineGroup gap="xs">
                          <IconUsers size={16} />
                          <Text size="sm">{group.memberCount} members</Text>
                        </MantineGroup>
                      </Card>
                    </Grid.Col>
                  ))}
                </Grid>
              )}
            </div>
          </Stack>
        </Container>

        <CreateGroupModal
          opened={createModalOpened}
          onClose={() => setCreateModalOpened(false)}
          onSuccess={loadGroups}
        />
      </AppLayout>
    </AuthGuard>
  );
}
