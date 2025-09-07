import { useEffect, useState } from 'react';
import { useRouter } from 'next/router';
import {
  Container,
  Title,
  Paper,
  Text,
  Group as MantineGroup,
  Badge,
  Tabs,
  Stack,
  Card,
  Button,
  Avatar,
  Table,
  ActionIcon
} from '@mantine/core';
import { AppLayout } from '../../components/AppLayout';
import { AuthGuard } from '../../components/AuthGuard';
import { IconUsers, IconCalendar, IconSettings, IconUserPlus } from '@tabler/icons-react';

interface GroupMember {
  id: string;
  name: string;
  slug: string;
  role: 'OWNER' | 'ADMIN' | 'MEMBER';
  joinedAt: string;
}

interface Group {
  id: string;
  name: string;
  slug: string;
  description: string | null;
  isPublic: boolean;
  createdAt: string;
  memberCount: number;
  members: GroupMember[];
}

export default function GroupPage() {
  const router = useRouter();
  const { slug } = router.query;
  const [group, setGroup] = useState<Group | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [userRole, setUserRole] = useState<'OWNER' | 'ADMIN' | 'MEMBER' | null>(null);

  useEffect(() => {
    if (slug) {
      loadGroupData();
    }
  }, [slug]);

  const loadGroupData = async () => {
    try {
      // TODO: Implement group API endpoint
      // const response = await fetch(`/api/groups/${slug}`);
      // const data = await response.json();
      // setGroup(data.group);

      // For now, load a mock group
      if (slug === 'test-group') {
        setGroup({
          id: 'group-1',
          name: 'Test Jump Group',
          slug: 'test-group',
          description: 'A test group for formation skydiving',
          isPublic: true,
          createdAt: new Date().toISOString(),
          memberCount: 1,
          members: [{
            id: 'admin-id',
            name: 'Admin User',
            slug: 'admin',
            role: 'OWNER',
            joinedAt: new Date().toISOString(),
          }],
        });
        setUserRole('OWNER');
      } else {
        setError('Group not found');
      }
      setLoading(false);
    } catch (err) {
      setError('Failed to load group');
      setLoading(false);
    }
  };

  const getRoleBadgeColor = (role: GroupMember['role']) => {
    switch (role) {
      case 'OWNER': return 'red';
      case 'ADMIN': return 'orange';
      default: return 'gray';
    }
  };

  if (loading) return (
    <AuthGuard>
      <AppLayout>
        <Container>
          <Text>Loading group...</Text>
        </Container>
      </AppLayout>
    </AuthGuard>
  );

  if (error || !group) return (
    <AuthGuard>
      <AppLayout>
        <Container>
          <Text c="red">{error || 'Group not found'}</Text>
        </Container>
      </AppLayout>
    </AuthGuard>
  );

  return (
    <AuthGuard>
      <AppLayout>
        <Container>
          <Paper p="xl" withBorder mb="xl">
            <MantineGroup justify="space-between" align="flex-start">
              <div>
                <MantineGroup mb="xs">
                  <Title order={2}>{group.name}</Title>
                  {group.isPublic ? (
                    <Badge color="green">Public</Badge>
                  ) : (
                    <Badge color="gray">Private</Badge>
                  )}
                </MantineGroup>
                {group.description && (
                  <Text c="dimmed" mb="xs">{group.description}</Text>
                )}
                <Text size="sm" c="dimmed">
                  Created {new Date(group.createdAt).toLocaleDateString()} • {group.memberCount} members
                </Text>
              </div>
              {userRole && (
                <Button leftSection={<IconUserPlus size={16} />}>
                  Invite Members
                </Button>
              )}
            </MantineGroup>
          </Paper>

          <Tabs defaultValue="members">
            <Tabs.List>
              <Tabs.Tab value="members" leftSection={<IconUsers size={16} />}>
                Members ({group.memberCount})
              </Tabs.Tab>
              <Tabs.Tab value="formations" leftSection={<IconCalendar size={16} />}>
                Formation Skydives
              </Tabs.Tab>
              {(userRole === 'OWNER' || userRole === 'ADMIN') && (
                <Tabs.Tab value="settings" leftSection={<IconSettings size={16} />}>
                  Settings
                </Tabs.Tab>
              )}
            </Tabs.List>

            <Tabs.Panel value="members" pt="md">
              <Paper p="md" withBorder>
                <Table>
                  <Table.Thead>
                    <Table.Tr>
                      <Table.Th>Member</Table.Th>
                      <Table.Th>Role</Table.Th>
                      <Table.Th>Joined</Table.Th>
                      {(userRole === 'OWNER' || userRole === 'ADMIN') && (
                        <Table.Th>Actions</Table.Th>
                      )}
                    </Table.Tr>
                  </Table.Thead>
                  <Table.Tbody>
                    {group.members.map((member) => (
                      <Table.Tr key={member.id}>
                        <Table.Td>
                          <MantineGroup>
                            <Avatar size="sm" radius="xl" color="accent">
                              {member.name.split(' ').map(n => n[0]).join('').toUpperCase()}
                            </Avatar>
                            <div>
                              <Text fw={500}>{member.name}</Text>
                              <Text size="xs" c="dimmed">@{member.slug}</Text>
                            </div>
                          </MantineGroup>
                        </Table.Td>
                        <Table.Td>
                          <Badge color={getRoleBadgeColor(member.role)}>
                            {member.role}
                          </Badge>
                        </Table.Td>
                        <Table.Td>
                          {new Date(member.joinedAt).toLocaleDateString()}
                        </Table.Td>
                        {(userRole === 'OWNER' || userRole === 'ADMIN') && (
                          <Table.Td>
                            <Text size="sm" c="dimmed">-</Text>
                          </Table.Td>
                        )}
                      </Table.Tr>
                    ))}
                  </Table.Tbody>
                </Table>
              </Paper>
            </Tabs.Panel>

            <Tabs.Panel value="formations" pt="md">
              <Card p="xl" withBorder>
                <Text c="dimmed" ta="center">No formation skydives recorded yet</Text>
              </Card>
            </Tabs.Panel>

            {(userRole === 'OWNER' || userRole === 'ADMIN') && (
              <Tabs.Panel value="settings" pt="md">
                <Paper p="md" withBorder>
                  <Title order={4} mb="md">Group Settings</Title>
                  <Text c="dimmed">Settings panel - to be implemented</Text>
                </Paper>
              </Tabs.Panel>
            )}
          </Tabs>
        </Container>
      </AppLayout>
    </AuthGuard>
  );
}
