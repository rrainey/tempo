// src/components/home/PendingConnectionsCard.tsx
import { Paper, Title, Stack, Group, Text, Button, Avatar, ActionIcon, Menu } from '@mantine/core';
import { IconCheck, IconX, IconDots } from '@tabler/icons-react';
import { useState, useEffect } from 'react';
import { notifications } from '@mantine/notifications';

interface ConnectionRequest {
  id: string;
  fromUser: {
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
  type: 'sent' | 'received';
}

interface PendingConnectionsCardProps {
  refreshKey?: number;
}

export function PendingConnectionsCard({ refreshKey = 0 }: PendingConnectionsCardProps) {
  const [sentRequests, setSentRequests] = useState<ConnectionRequest[]>([]);
  const [receivedRequests, setReceivedRequests] = useState<ConnectionRequest[]>([]);
  const [loading, setLoading] = useState(true);
  const [processingId, setProcessingId] = useState<string | null>(null);

  useEffect(() => {
    loadPendingConnections();
  }, [refreshKey]);

  const loadPendingConnections = async () => {
    try {
      const response = await fetch('/api/connections');
      if (response.ok) {
        const data = await response.json();
        setSentRequests(data.pendingRequests.sent.map((r: any) => ({ ...r, type: 'sent' })));
        setReceivedRequests(data.pendingRequests.received.map((r: any) => ({ ...r, type: 'received' })));
      }
    } catch (error) {
      console.error('Error loading pending connections:', error);
    } finally {
      setLoading(false);
    }
  };

  const handleResponse = async (requestId: string, action: 'accept' | 'decline') => {
    setProcessingId(requestId);
    
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

      // Remove from received requests
      setReceivedRequests(prev => prev.filter(req => req.id !== requestId));
    } catch (error) {
      notifications.show({
        title: 'Error',
        message: error instanceof Error ? error.message : 'An error occurred',
        color: 'red',
      });
    } finally {
      setProcessingId(null);
    }
  };

  const cancelRequest = async (requestId: string) => {
    setProcessingId(requestId);
    
    try {
      const response = await fetch(`/api/connections/${requestId}`, {
        method: 'DELETE'
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.error || 'Failed to cancel request');
      }

      notifications.show({
        title: 'Success',
        message: 'Connection request cancelled',
        color: 'green',
      });

      // Remove from sent requests
      setSentRequests(prev => prev.filter(req => req.id !== requestId));
    } catch (error) {
      notifications.show({
        title: 'Error',
        message: error instanceof Error ? error.message : 'An error occurred',
        color: 'red',
      });
    } finally {
      setProcessingId(null);
    }
  };

  const hasAnyRequests = receivedRequests.length > 0 || sentRequests.length > 0;

  if (loading || !hasAnyRequests) {
    return null;
  }

  return (
    <Paper p="md" withBorder mb="xl" style={{ backgroundColor: 'var(--mantine-color-dark-8)' }}>
      <Title order={4} mb="md">Connection Requests</Title>
      
      <Stack gap="md">
        {/* Received Requests */}
        {receivedRequests.length > 0 && (
          <>
            <Text size="sm" c="dimmed" fw={500}>Received</Text>
            <Stack gap="sm">
              {receivedRequests.map((request) => (
                <Group key={request.id} justify="space-between">
                  <Group>
                    <Avatar name={request.fromUser.name} color="initials" size="sm" />
                    <div>
                      <Text fw={500}>{request.fromUser.name}</Text>
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
                      leftSection={<IconCheck size={14} />}
                      loading={processingId === request.id}
                      onClick={() => handleResponse(request.id, 'accept')}
                    >
                      Accept
                    </Button>
                    <Button
                      size="xs"
                      color="red"
                      variant="subtle"
                      leftSection={<IconX size={14} />}
                      loading={processingId === request.id}
                      onClick={() => handleResponse(request.id, 'decline')}
                    >
                      Decline
                    </Button>
                  </Group>
                </Group>
              ))}
            </Stack>
          </>
        )}

        {/* Sent Requests */}
        {sentRequests.length > 0 && (
          <>
            <Text size="sm" c="dimmed" fw={500}>Sent</Text>
            <Stack gap="sm">
              {sentRequests.map((request) => (
                <Group key={request.id} justify="space-between">
                  <Group>
                    <Avatar name={request.toUser!.name} color="initials" size="sm" />
                    <div>
                      <Text fw={500}>To: {request.toUser!.name}</Text>
                      {request.message && (
                        <Text size="sm" c="dimmed">{request.message}</Text>
                      )}
                      <Text size="xs" c="dimmed">
                        {new Date(request.createdAt).toLocaleDateString()} · Pending
                      </Text>
                    </div>
                  </Group>
                  <Menu position="bottom-end" withinPortal>
                    <Menu.Target>
                      <ActionIcon variant="subtle" size="sm">
                        <IconDots size={16} />
                      </ActionIcon>
                    </Menu.Target>
                    <Menu.Dropdown>
                      <Menu.Item
                        color="red"
                        onClick={() => cancelRequest(request.id)}
                        disabled={processingId === request.id}
                      >
                        Cancel Request
                      </Menu.Item>
                    </Menu.Dropdown>
                  </Menu>
                </Group>
              ))}
            </Stack>
          </>
        )}
      </Stack>
    </Paper>
  );
}