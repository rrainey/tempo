// src/pages/connections.tsx
import { useState, useEffect } from 'react';
import { Container, Title, Paper, Group, Text, Avatar, ActionIcon, Menu, Stack, Loader, Center, Button, Badge } from '@mantine/core';
import { modals } from '@mantine/modals';
import { notifications } from '@mantine/notifications';
import { IconDotsVertical, IconUserX, IconUserPlus, IconSearch, IconTrash } from '@tabler/icons-react';
import { AppLayout } from '../components/AppLayout';
import { AuthGuard } from '../components/AuthGuard';
import { UserSearchModal } from '../components/UserSearchModal';
import { useRouter } from 'next/router';
import Link from 'next/link';

interface Connection {
  id: string;
  user: {
    id: string;
    name: string;
    email: string;
    slug: string;
  };
  createdAt: string;
}

interface PendingRequest {
  id: string;
  fromUser?: {
    id: string;
    name: string;
    slug: string;
    email: string;
  };
  toUser?: {
    id: string;
    name: string;
    slug: string;
    email: string;
  };
  message?: string;
  createdAt: string;
}

interface PendingInvitation {
  id: string;
  code: string;
  email: string;
  group: {
    id: string;
    name: string;
    slug: string;
  } | null;
  groupRole: string | null;
  userRole: string;
  expiresAt: string;
  createdAt: string;
}

export default function ConnectionsPage() {
  const router = useRouter();
  const [connections, setConnections] = useState<Connection[]>([]);
  const [sentRequests, setSentRequests] = useState<PendingRequest[]>([]);
  const [receivedRequests, setReceivedRequests] = useState<PendingRequest[]>([]);
  const [pendingInvitations, setPendingInvitations] = useState<PendingInvitation[]>([]);
  const [loading, setLoading] = useState(true);
  const [searchModalOpen, setSearchModalOpen] = useState(false);

  useEffect(() => {
    loadConnections();
    loadPendingInvitations();
  }, []);

  const loadConnections = async () => {
    try {
      const response = await fetch('/api/connections');
      if (response.ok) {
        const data = await response.json();
        setConnections(data.connections);
        setSentRequests(data.pendingRequests.sent);
        setReceivedRequests(data.pendingRequests.received);
      }
    } catch (error) {
      console.error('Error loading connections:', error);
      notifications.show({
        title: 'Error',
        message: 'Failed to load connections',
        color: 'red',
      });
    } finally {
      setLoading(false);
    }
  };

  const loadPendingInvitations = async () => {
    try {
      const response = await fetch('/api/invitations/pending');
      if (response.ok) {
        const data = await response.json();
        setPendingInvitations(data.invitations || []);
      }
    } catch (error) {
      console.error('Error loading pending invitations:', error);
    }
  };

  const handleRemoveConnection = (connectionId: string, userName: string) => {
    modals.openConfirmModal({
      title: 'Remove Connection',
      children: (
        <Text size="sm">
          Are you sure you want to remove your connection with <strong>{userName}</strong>? 
          They will no longer be able to see your jumps marked as &quot;visible to connections&quot;.
        </Text>
      ),
      labels: { confirm: 'Remove Connection', cancel: 'Cancel' },
      confirmProps: { color: 'red' },
      onConfirm: async () => {
        try {
          const response = await fetch('/api/connections', {
            method: 'DELETE',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ connectionId })
          });

          if (!response.ok) {
            throw new Error('Failed to remove connection');
          }

          notifications.show({
            title: 'Success',
            message: 'Connection removed',
            color: 'green',
          });

          // Remove from state
          setConnections(prev => prev.filter(conn => conn.id !== connectionId));
        } catch (error) {
          notifications.show({
            title: 'Error',
            message: 'Failed to remove connection',
            color: 'red',
          });
        }
      }
    });
  };

  const handleCancelRequest = async (requestId: string) => {
    try {
      const response = await fetch(`/api/connections/${requestId}`, {
        method: 'DELETE'
      });

      if (!response.ok) {
        throw new Error('Failed to cancel request');
      }

      notifications.show({
        title: 'Success',
        message: 'Connection request cancelled',
        color: 'green',
      });

      setSentRequests(prev => prev.filter(req => req.id !== requestId));
    } catch (error) {
      notifications.show({
        title: 'Error',
        message: 'Failed to cancel request',
        color: 'red',
      });
    }
  };

  const handleRespondToRequest = async (requestId: string, action: 'accept' | 'decline') => {
    try {
      const response = await fetch(`/api/connections/${requestId}/respond`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ action })
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || `Failed to ${action} request`);
      }

      notifications.show({
        title: 'Success',
        message: data.message,
        color: 'green',
      });

      // Reload connections to get the updated list
      loadConnections();
    } catch (error) {
      notifications.show({
        title: 'Error',
        message: error instanceof Error ? error.message : 'An error occurred',
        color: 'red',
      });
    }
  };

  const handleDeleteInvitation = (invitationCode: string, email: string) => {
    modals.openConfirmModal({
      title: 'Delete Invitation',
      children: (
        <Text size="sm">
          Are you sure you want to delete the invitation for <strong>{email}</strong>? 
          They will no longer be able to use this invitation link to join.
        </Text>
      ),
      labels: { confirm: 'Delete Invitation', cancel: 'Cancel' },
      confirmProps: { color: 'red' },
      onConfirm: async () => {
        try {
          const response = await fetch(`/api/invitations/${invitationCode}/delete`, {
            method: 'DELETE'
          });

          if (!response.ok) {
            const data = await response.json();
            throw new Error(data.error || 'Failed to delete invitation');
          }

          notifications.show({
            title: 'Success',
            message: 'Invitation deleted successfully',
            color: 'green',
          });

          // Remove from state
          setPendingInvitations(prev => prev.filter(inv => inv.code !== invitationCode));
        } catch (error) {
          notifications.show({
            title: 'Error',
            message: error instanceof Error ? error.message : 'Failed to delete invitation',
            color: 'red',
          });
        }
      }
    });
  };

  if (loading) {
    return (
      <AuthGuard>
        <AppLayout>
          <Center h={400}>
            <Loader />
          </Center>
        </AppLayout>
      </AuthGuard>
    );
  }

  return (
    <AuthGuard>
      <AppLayout>
        <Container size="md">
          <Group justify="space-between" mb="xl">
            <Group>
              <Title order={2}>Connections</Title>
              <Badge size="lg" variant="filled">
                {connections.length} {connections.length === 1 ? 'Connection' : 'Connections'}
              </Badge>
            </Group>
            <Button
              leftSection={<IconSearch size={16} />}
              onClick={() => setSearchModalOpen(true)}
            >
              Find Users
            </Button>
          </Group>

          {/* Pending Received Requests */}
          {receivedRequests.length > 0 && (
            <Paper p="md" mb="xl" withBorder>
              <Title order={4} mb="md">Pending Connection Requests</Title>
              <Stack gap="sm">
                {receivedRequests.map((request) => (
                  <Group key={request.id} justify="space-between">
                    <Group>
                      <Avatar name={request.fromUser!.name} color="initials" />
                      <div>
                        <Text fw={500}>
                          <Link href={`/users/${request.fromUser!.slug}`} style={{ color: 'inherit', textDecoration: 'none' }}>
                            {request.fromUser!.name}
                          </Link>
                        </Text>
                        {request.message && (
                          <Text size="sm" c="dimmed">{request.message}</Text>
                        )}
                        <Text size="xs" c="dimmed">
                          {new Date(request.createdAt).toLocaleDateString()}
                        </Text>
                      </div>
                    </Group>
                    <Group gap="xs">
                      <Button
                        size="xs"
                        onClick={() => handleRespondToRequest(request.id, 'accept')}
                      >
                        Accept
                      </Button>
                      <Button
                        size="xs"
                        color="red"
                        variant="subtle"
                        onClick={() => handleRespondToRequest(request.id, 'decline')}
                      >
                        Decline
                      </Button>
                    </Group>
                  </Group>
                ))}
              </Stack>
            </Paper>
          )}

          {/* Pending Invitations (Created by User) */}
          {pendingInvitations.length > 0 && (
            <Paper p="md" mb="xl" withBorder>
              <Title order={4} mb="md">Pending Invitations</Title>
              <Text size="sm" c="dimmed" mb="md">
                Invitations you&apos;ve sent that haven&apos;t been accepted yet
              </Text>
              <Stack gap="sm">
                {pendingInvitations.map((invitation) => (
                  <Group key={invitation.id} justify="space-between">
                    <div>
                      <Group gap="xs">
                        <Text fw={500}>{invitation.email}</Text>
                        {invitation.group && (
                          <Badge size="sm" variant="light">
                            {invitation.group.name}
                          </Badge>
                        )}
                      </Group>
                      <Text size="xs" c="dimmed">
                        Expires: {new Date(invitation.expiresAt).toLocaleDateString()}
                      </Text>
                      {invitation.groupRole && (
                        <Text size="xs" c="dimmed">
                          Role: {invitation.groupRole}
                        </Text>
                      )}
                    </div>
                    <ActionIcon
                      color="red"
                      variant="subtle"
                      onClick={() => handleDeleteInvitation(invitation.code, invitation.email)}
                    >
                      <IconTrash size={16} />
                    </ActionIcon>
                  </Group>
                ))}
              </Stack>
            </Paper>
          )}

          {/* Active Connections */}
          <Paper p="md" mb="xl" withBorder>
            <Title order={4} mb="md">My Connections</Title>
            {connections.length === 0 ? (
              <Text c="dimmed" ta="center" py="xl">
                You don&apos;t have any connections yet. Find and connect with other jumpers!
              </Text>
            ) : (
              <Stack gap="sm">
                {connections.map((connection) => (
                  <Group key={connection.id} justify="space-between">
                    <Group>
                      <Avatar name={connection.user.name} color="initials" />
                      <div>
                        <Text fw={500}>
                          <Link href={`/users/${connection.user.slug}`} style={{ color: 'inherit', textDecoration: 'none' }}>
                            {connection.user.name}
                          </Link>
                        </Text>
                        <Text size="xs" c="dimmed">
                          Connected since {new Date(connection.createdAt).toLocaleDateString()}
                        </Text>
                      </div>
                    </Group>
                    <Menu position="bottom-end" withinPortal>
                      <Menu.Target>
                        <ActionIcon variant="subtle">
                          <IconDotsVertical size={16} />
                        </ActionIcon>
                      </Menu.Target>
                      <Menu.Dropdown>
                        <Menu.Item
                          component={Link}
                          href={`/users/${connection.user.slug}`}
                          leftSection={<IconUserPlus size={14} />}
                        >
                          View Profile
                        </Menu.Item>
                        <Menu.Item
                          color="red"
                          leftSection={<IconUserX size={14} />}
                          onClick={() => handleRemoveConnection(connection.id, connection.user.name)}
                        >
                          Remove Connection
                        </Menu.Item>
                      </Menu.Dropdown>
                    </Menu>
                  </Group>
                ))}
              </Stack>
            )}
          </Paper>

          {/* Pending Sent Requests */}
          {sentRequests.length > 0 && (
            <Paper p="md" withBorder>
              <Title order={4} mb="md">Sent Connection Requests</Title>
              <Stack gap="sm">
                {sentRequests.map((request) => (
                  <Group key={request.id} justify="space-between">
                    <Group>
                      <Avatar name={request.toUser!.name} color="initials" />
                      <div>
                        <Text fw={500}>
                          <Link href={`/users/${request.toUser!.slug}`} style={{ color: 'inherit', textDecoration: 'none' }}>
                            {request.toUser!.name}
                          </Link>
                        </Text>
                        {request.message && (
                          <Text size="sm" c="dimmed">{request.message}</Text>
                        )}
                        <Text size="xs" c="dimmed">
                          Sent {new Date(request.createdAt).toLocaleDateString()}
                        </Text>
                      </div>
                    </Group>
                    <Menu position="bottom-end" withinPortal>
                      <Menu.Target>
                        <ActionIcon variant="subtle">
                          <IconDotsVertical size={16} />
                        </ActionIcon>
                      </Menu.Target>
                      <Menu.Dropdown>
                        <Menu.Item
                          color="red"
                          onClick={() => handleCancelRequest(request.id)}
                        >
                          Cancel Request
                        </Menu.Item>
                      </Menu.Dropdown>
                    </Menu>
                  </Group>
                ))}
              </Stack>
            </Paper>
          )}
        </Container>

        <UserSearchModal
          opened={searchModalOpen}
          onClose={() => {
            setSearchModalOpen(false);
            loadConnections(); // Refresh connections after closing
          }}
        />
      </AppLayout>
    </AuthGuard>
  );
}