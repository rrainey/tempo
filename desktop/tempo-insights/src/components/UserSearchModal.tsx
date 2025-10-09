// src/components/UserSearchModal.tsx
import { Modal, TextInput, Stack, Text, Group, Avatar, Button, Loader, Center, Badge, Paper } from '@mantine/core';
import { useDebouncedValue } from '@mantine/hooks';
import { useState, useEffect } from 'react';
import { IconSearch, IconUserPlus, IconCheck, IconClock } from '@tabler/icons-react';
import { SendConnectionModal } from './SendConnectionModal';

interface User {
  id: string;
  name: string;
  email: string;
  slug: string;
  connectionStatus: 'connected' | 'request_sent' | 'request_received' | 'none';
}

interface UserSearchModalProps {
  opened: boolean;
  onClose: () => void;
}

export function UserSearchModal({ opened, onClose }: UserSearchModalProps) {
  const [searchTerm, setSearchTerm] = useState('');
  const [debouncedSearch] = useDebouncedValue(searchTerm, 300);
  const [users, setUsers] = useState<User[]>([]);
  const [loading, setLoading] = useState(false);
  const [selectedUser, setSelectedUser] = useState<User | null>(null);
  const [sendConnectionModalOpen, setSendConnectionModalOpen] = useState(false);

  useEffect(() => {
    if (debouncedSearch) {
      searchUsers();
    } else {
      setUsers([]);
    }
  }, [debouncedSearch]);

  const searchUsers = async () => {
    setLoading(true);
    try {
      const response = await fetch(`/api/users/search?q=${encodeURIComponent(debouncedSearch)}`);
      if (response.ok) {
        const data = await response.json();
        setUsers(data.users);
      }
    } catch (error) {
      console.error('Error searching users:', error);
    } finally {
      setLoading(false);
    }
  };

  const handleConnect = (user: User) => {
    setSelectedUser(user);
    setSendConnectionModalOpen(true);
  };

  const handleConnectionSent = () => {
    // Refresh the search to update connection status
    if (searchTerm) {
      searchUsers();
    }
  };

  const getStatusBadge = (status: string) => {
    switch (status) {
      case 'connected':
        return <Badge color="green" size="sm" leftSection={<IconCheck size={12} />}>Connected</Badge>;
      case 'request_sent':
        return <Badge color="blue" size="sm" leftSection={<IconClock size={12} />}>Request Sent</Badge>;
      case 'request_received':
        return <Badge color="yellow" size="sm">Request Received</Badge>;
      default:
        return null;
    }
  };

  return (
    <>
      <Modal
        opened={opened}
        onClose={onClose}
        title="Find Users"
        size="md"
      >
        <Stack>
          <TextInput
            placeholder="Search by name or email..."
            leftSection={<IconSearch size={16} />}
            value={searchTerm}
            onChange={(e) => setSearchTerm(e.currentTarget.value)}
            autoFocus
          />

          {loading && (
            <Center py="xl">
              <Loader size="sm" />
            </Center>
          )}

          {!loading && searchTerm && users.length === 0 && (
            <Text c="dimmed" ta="center" py="xl">
              No users found matching &quot;{searchTerm}&quot;
            </Text>
          )}

          {!loading && users.length > 0 && (
            <Stack gap="xs">
              {users.map((user) => (
                <Paper key={user.id} p="sm" withBorder>
                  <Group justify="space-between">
                    <Group>
                      <Avatar name={user.name} color="initials" />
                      <div>
                        <Text fw={500}>{user.name}</Text>
                        <Text size="sm" c="dimmed">{user.email}</Text>
                      </div>
                    </Group>
                    {user.connectionStatus === 'none' ? (
                      <Button
                        size="xs"
                        leftSection={<IconUserPlus size={14} />}
                        onClick={() => handleConnect(user)}
                      >
                        Connect
                      </Button>
                    ) : (
                      getStatusBadge(user.connectionStatus)
                    )}
                  </Group>
                </Paper>
              ))}
            </Stack>
          )}
        </Stack>
      </Modal>

      {selectedUser && (
        <SendConnectionModal
          opened={sendConnectionModalOpen}
          onClose={() => {
            setSendConnectionModalOpen(false);
            setSelectedUser(null);
          }}
          targetUser={selectedUser}
          onSuccess={handleConnectionSent}
        />
      )}
    </>
  );
}