// pages/groups/[slug].tsx
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
  Card,
  Button,
  Avatar,
  Table,
  Loader,
  Center,
  Modal,
  Stack,
  Select,
  Divider
} from '@mantine/core';
import { AppLayout } from '../../components/AppLayout';
import { AuthGuard } from '../../components/AuthGuard';
import { InviteMemberModal } from '../../components/InviteMemberModal';
import { notifications } from '@mantine/notifications';
import { IconUsers, IconCalendar, IconSettings, IconUserPlus, IconUserCheck, IconTrash, IconAlertTriangle } from '@tabler/icons-react';

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
  const [deleteModalOpened, setDeleteModalOpened] = useState(false);
  const [reassignModalOpened, setReassignModalOpened] = useState(false);
  const [deleting, setDeleting] = useState(false);
  const [selectedNewAdmin, setSelectedNewAdmin] = useState<string | null>(null);
  const [promoting, setPromoting] = useState(false);
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
      setError('Failed to load group: ' + (err instanceof Error ? err.message : 'Unknown error'));
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

  const handleDeleteGroup = async () => {
    if (!group) return;

    setDeleting(true);

    try {
      const response = await fetch(`/api/groups/${group.slug}/delete`, {
        method: 'DELETE',
      });

      const data = await response.json();

      if (!response.ok) {
        // Check if we need to reassign admin
        if (data.requiresAdminReassignment) {
          setDeleteModalOpened(false);
          setReassignModalOpened(true);
          notifications.show({
            title: 'Action Required',
            message: data.message,
            color: 'orange',
          });
          return;
        }
        throw new Error(data.error || 'Failed to delete group');
      }

      notifications.show({
        title: 'Success',
        message: 'Group deleted successfully',
        color: 'green',
      });

      // Redirect to groups page
      router.push('/groups');
    } catch (error) {
      notifications.show({
        title: 'Error',
        message: error instanceof Error ? error.message : 'Failed to delete group',
        color: 'red',
      });
    } finally {
      setDeleting(false);
    }
  };

  const handlePromoteMember = async () => {
    if (!group || !selectedNewAdmin) return;

    setPromoting(true);

    try {
      const response = await fetch(`/api/groups/${group.slug}/members/${selectedNewAdmin}/promote`, {
        method: 'POST',
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Failed to promote member');
      }

      notifications.show({
        title: 'Success',
        message: 'Member promoted to admin successfully',
        color: 'green',
      });

      // Reload group data
      await loadGroupData();
      
      // Close reassign modal and open delete confirmation
      setReassignModalOpened(false);
      setSelectedNewAdmin(null);
      
      notifications.show({
        title: 'Admin Assigned',
        message: 'You can now delete the group',
        color: 'blue',
      });

      // Open delete modal after promotion
      setTimeout(() => setDeleteModalOpened(true), 500);
    } catch (error) {
      notifications.show({
        title: 'Error',
        message: error instanceof Error ? error.message : 'Failed to promote member',
        color: 'red',
      });
    } finally {
      setPromoting(false);
    }
  };

  const getRoleBadgeColor = (role: GroupMember['role']) => {
    switch (role) {
      case 'OWNER': return 'red';
      case 'ADMIN': return 'orange';
      default: return 'gray';
    }
  };

  // Get eligible members for promotion (non-admins)
  const eligibleMembers = group?.members.filter(
    m => m.role === 'MEMBER'
  ) || [];

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
                <Stack gap="md">
                  <Paper p="md" withBorder>
                    <Title order={4} mb="md">Group Information</Title>
                    <Stack gap="xs">
                      <MantineGroup justify="space-between">
                        <Text size="sm" c="dimmed">Group Name</Text>
                        <Text size="sm" fw={500}>{group.name}</Text>
                      </MantineGroup>
                      <MantineGroup justify="space-between">
                        <Text size="sm" c="dimmed">Visibility</Text>
                        <Text size="sm">{group.isPublic ? 'Public' : 'Private'}</Text>
                      </MantineGroup>
                      <MantineGroup justify="space-between">
                        <Text size="sm" c="dimmed">Total Members</Text>
                        <Text size="sm">{group.memberCount}</Text>
                      </MantineGroup>
                      <MantineGroup justify="space-between">
                        <Text size="sm" c="dimmed">Created</Text>
                        <Text size="sm">{new Date(group.createdAt).toLocaleDateString()}</Text>
                      </MantineGroup>
                    </Stack>
                  </Paper>

                  <Paper p="md" withBorder>
                    <Title order={4} mb="md" c="red">Danger Zone</Title>
                    <Card withBorder p="md" style={{ backgroundColor: 'var(--mantine-color-dark-8)' }}>
                      <MantineGroup justify="space-between">
                        <div>
                          <Text fw={500} mb="xs">Delete this group</Text>
                          <Text size="sm" c="dimmed">
                            Once you delete a group, there is no going back. All members will be removed.
                          </Text>
                        </div>
                        <Button
                          color="red"
                          variant="light"
                          leftSection={<IconTrash size={16} />}
                          onClick={() => setDeleteModalOpened(true)}
                        >
                          Delete Group
                        </Button>
                      </MantineGroup>
                    </Card>
                  </Paper>
                </Stack>
              </Tabs.Panel>
            )}
          </Tabs>

          <InviteMemberModal
            opened={inviteModalOpened}
            onClose={() => setInviteModalOpened(false)}
            groupSlug={group.slug}
            userRole={group.userRole || 'MEMBER'}
          />

          {/* Delete Confirmation Modal */}
          <Modal
            opened={deleteModalOpened}
            onClose={() => setDeleteModalOpened(false)}
            title="Delete Group"
            size="md"
          >
            <Stack gap="md">
              <MantineGroup gap="sm">
                <IconAlertTriangle size={24} color="red" />
                <div style={{ flex: 1 }}>
                  <Text fw={500}>Are you sure you want to delete "{group.name}"?</Text>
                  <Text size="sm" c="dimmed" mt="xs">
                    This action cannot be undone. This will permanently:
                  </Text>
                </div>
              </MantineGroup>

              <Card withBorder p="md" style={{ backgroundColor: 'var(--mantine-color-dark-8)' }}>
                <Stack gap="xs">
                  <Text size="sm">• Remove all {group.memberCount} members from the group</Text>
                  <Text size="sm">• Delete all pending invitations</Text>
                  <Text size="sm">• Remove group associations from formation skydives</Text>
                  <Text size="sm" c="dimmed" mt="xs">
                    Note: Individual jump logs and formation data will be preserved.
                  </Text>
                </Stack>
              </Card>

              <MantineGroup justify="flex-end" gap="xs" mt="md">
                <Button
                  variant="subtle"
                  onClick={() => setDeleteModalOpened(false)}
                  disabled={deleting}
                >
                  Cancel
                </Button>
                <Button
                  color="red"
                  leftSection={<IconTrash size={16} />}
                  onClick={handleDeleteGroup}
                  loading={deleting}
                >
                  Delete Permanently
                </Button>
              </MantineGroup>
            </Stack>
          </Modal>

          {/* Reassign Admin Modal */}
          <Modal
            opened={reassignModalOpened}
            onClose={() => {
              setReassignModalOpened(false);
              setSelectedNewAdmin(null);
            }}
            title="Assign New Admin"
            size="md"
          >
            <Stack gap="md">
              <Text size="sm">
                You are the last admin of this group. Please promote another member to admin 
                before deleting the group, or remove all other members first.
              </Text>

              {eligibleMembers.length > 0 ? (
                <>
                  <Select
                    label="Select a member to promote to admin"
                    placeholder="Choose a member"
                    data={eligibleMembers.map(m => ({
                      value: m.id,
                      label: `${m.name} (@${m.slug})`
                    }))}
                    value={selectedNewAdmin}
                    onChange={setSelectedNewAdmin}
                  />

                  <Divider />

                  <Text size="sm" c="dimmed">
                    After promoting a new admin, you can proceed with deleting the group.
                  </Text>

                  <MantineGroup justify="flex-end" gap="xs">
                    <Button
                      variant="subtle"
                      onClick={() => {
                        setReassignModalOpened(false);
                        setSelectedNewAdmin(null);
                      }}
                      disabled={promoting}
                    >
                      Cancel
                    </Button>
                    <Button
                      onClick={handlePromoteMember}
                      loading={promoting}
                      disabled={!selectedNewAdmin}
                    >
                      Promote to Admin
                    </Button>
                  </MantineGroup>
                </>
              ) : (
                <>
                  <Text size="sm" c="dimmed">
                    There are no regular members to promote. You must remove all other members 
                    before you can delete this group.
                  </Text>
                  <MantineGroup justify="flex-end">
                    <Button
                      variant="subtle"
                      onClick={() => setReassignModalOpened(false)}
                    >
                      Close
                    </Button>
                  </MantineGroup>
                </>
              )}
            </Stack>
          </Modal>
        </Container>
      </AppLayout>
    </AuthGuard>
  );
}