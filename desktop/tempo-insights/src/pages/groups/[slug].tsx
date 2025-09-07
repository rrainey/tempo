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
  Loader,
  Center
} from '@mantine/core';
import { AppLayout } from '../../components/AppLayout';
import { AuthGuard } from '../../components/AuthGuard';
import { InviteMemberModal } from '../../components/InviteMemberModal';
import { notifications } from '@mantine/notifications';
import { IconUsers, IconCalendar, IconSettings, IconUserPlus, IconUserCheck } from '@tabler/icons-react';

interface GroupMember {
  id: string;
  name: string;
  slug: string;
  email: string;
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
  userRole: 'OWNER' | 'ADMIN' | 'MEMBER' | null;
  isMember: boolean;
}

export default function GroupPage() {
  const router = useRouter();
  const { slug } = router.query;
  const [group, setGroup] = useState<Group | null>(null);
  const [loading, setLoading] = useState(true);
  const [joining, setJoining] = useState(false);
  const [inviteModalOpened, setInviteModalOpened] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (slug) {
      loadGroupData();
    }
  }, [slug]);

  const loadGroupData = async () => {
    try {
      setLoading(true);
      const response = await fetch(`/api/groups/${slug}`);

      if (!response.ok) {
        if (response.status === 404) {
          setError('Group not found');
        } else {
          setError('Failed to load group');
        }
        return;
      }

      const data = await response.json();
      setGroup(data.group);
    } catch (err) {
      setError('Failed to load group');
    } finally {
      setLoading(false);
    }
  };

  const handleJoinGroup = async () => {
    if (!group) return;

    setJoining(true);
    try {
      const response = await fetch(`/api/groups/${group.slug}/join`, {
        method: 'POST',
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Failed to join group');
      }

      notifications.show({
        title: 'Success',
        message: 'Successfully joined group!',
        color: 'green',
      });

      await loadGroupData();
    } catch (error) {
      notifications.show({
        title: 'Error',
        message: error instanceof Error ? error.message : 'Failed to join group',
        color: 'red',
      });
    } finally {
      setJoining(false);
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
        <Center style={{ height: '50vh' }}>
          <Loader size="lg" color="accent" />
        </Center>
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
              <div>
                {!group.isMember && group.isPublic && (
                  <Button
                    leftSection={<IconUserCheck size={16} />}
                    onClick={handleJoinGroup}
                    loading={joining}
                    disabled={joining}
                  >
                    Join Group
                  </Button>
                )}
                {group.userRole && (group.userRole === 'OWNER' || group.userRole === 'ADMIN') && (
                  <Button
                    leftSection={<IconUserPlus size={16} />}
                    onClick={() => setInviteModalOpened(true)}
                  >
                    Invite Members
                  </Button>
                )}
              </div>
            </MantineGroup>
          </Paper>

          <Tabs defaultValue="members">
            <Tabs.List>
              <Tabs.Tab value="members" leftSection={<IconUsers size={16} />}>
                Members ({group.memberCount})
              </Tabs.Tab>
              {group.isMember && (
                <Tabs.Tab value="formations" leftSection={<IconCalendar size={16} />}>
                  Formation Skydives
                </Tabs.Tab>
              )}
              {(group.userRole === 'OWNER' || group.userRole === 'ADMIN') && (
                <Tabs.Tab value="settings" leftSection={<IconSettings size={16} />}>
                  Settings
                </Tabs.Tab>
              )}
            </Tabs.List>

            <Tabs.Panel value="members" pt="md">
              {!group.isMember && !group.isPublic ? (
                <Card p="xl" withBorder>
                  <Text c="dimmed" ta="center">
                    This is a private group. You need an invitation to view members.
                  </Text>
                </Card>
              ) : (
                <Paper p="md" withBorder>
                  <Table>
                    <Table.Thead>
                      <Table.Tr>
                        <Table.Th>Member</Table.Th>
                        <Table.Th>Role</Table.Th>
                        <Table.Th>Joined</Table.Th>
                        {(group.userRole === 'OWNER' || group.userRole === 'ADMIN') && (
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
                          {(group.userRole === 'OWNER' || group.userRole === 'ADMIN') && (
                            <Table.Td>
                              <Text size="sm" c="dimmed">-</Text>
                            </Table.Td>
                          )}
                        </Table.Tr>
                      ))}
                    </Table.Tbody>
                  </Table>
                </Paper>
              )}
            </Tabs.Panel>

            {group.isMember && (
              <Tabs.Panel value="formations" pt="md">
                <Card p="xl" withBorder>
                  <Text c="dimmed" ta="center">No formation skydives recorded yet</Text>
                </Card>
              </Tabs.Panel>
            )}

            {(group.userRole === 'OWNER' || group.userRole === 'ADMIN') && (
              <Tabs.Panel value="settings" pt="md">
                <Paper p="md" withBorder>
                  <Title order={4} mb="md">Group Settings</Title>
                  <Text c="dimmed">Settings panel - to be implemented</Text>
                </Paper>
              </Tabs.Panel>
            )}
          </Tabs>

          <InviteMemberModal
            opened={inviteModalOpened}
            onClose={() => setInviteModalOpened(false)}
            groupSlug={group.slug}
            userRole={group.userRole || 'MEMBER'}
          />
        </Container>
      </AppLayout>
    </AuthGuard>
  );
}
