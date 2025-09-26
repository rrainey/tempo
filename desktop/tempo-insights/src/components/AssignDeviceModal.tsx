import { Modal, Select, NumberInput, Button, Stack, Text, Group } from '@mantine/core';
import { useForm } from '@mantine/form';
import { notifications } from '@mantine/notifications';
import { useState, useEffect } from 'react';

interface AssignDeviceForm {
  userId: string;
  nextJumpNumber: number;
}

interface User {
  id: string;
  name: string;
  email: string;
  slug: string;
  nextJumpNumber: number;
}

interface AssignDeviceModalProps {
  opened: boolean;
  onClose: () => void;
  device: {
    id: string;
    name: string;
    owner?: {
      name: string;
    };
  } | null;
  onSuccess?: (userId: string) => void;
}

export function AssignDeviceModal({ opened, onClose, device, onSuccess }: AssignDeviceModalProps) {
  const [loading, setLoading] = useState(false);
  const [users, setUsers] = useState<User[]>([]);
  const [loadingUsers, setLoadingUsers] = useState(true);
  const [selectedUser, setSelectedUser] = useState<User | null>(null);

  const form = useForm<AssignDeviceForm>({
    initialValues: {
      userId: '',
      nextJumpNumber: 1,
    },
    validate: {
      userId: (value) => {
        if (!value) return 'Please select a user';
        return null;
      },
      nextJumpNumber: (value) => {
        if (!value || value < 1) return 'Jump number must be at least 1';
        return null;
      },
    },
  });

  useEffect(() => {
    if (opened) {
      loadUsers();
      form.reset();
      setSelectedUser(null);
    }
  }, [opened]);

  const loadUsers = async () => {
    try {
      setLoadingUsers(true);
      const response = await fetch('/api/users?limit=100');
      if (response.ok) {
        const data = await response.json();
        setUsers(data.users || []);
      } else {
        console.error('Failed to load users');
        notifications.show({
          title: 'Error',
          message: 'Failed to load users',
          color: 'red',
        });
      }
    } catch (error) {
      console.error('Error loading users:', error);
      notifications.show({
        title: 'Error',
        message: 'Failed to load users',
        color: 'red',
      });
    } finally {
      setLoadingUsers(false);
    }
  };

  const handleUserSelect = (userId: string) => {
    const user = users.find(u => u.id === userId);
    if (user) {
      setSelectedUser(user);
      // Set the user's next jump number as the default
      form.setFieldValue('nextJumpNumber', user.nextJumpNumber || 1);
    }
  };

  const handleSubmit = async (values: AssignDeviceForm) => {
    if (!device) return;

    setLoading(true);
    try {
      // Call the parent's onSuccess handler which will make the API call
      await onSuccess?.(values.userId);
      
      form.reset();
      onClose();
    } catch (error) {
      // Error handling is done in the parent component
      console.error('Error in assign modal:', error);
    } finally {
      setLoading(false);
    }
  };

  if (!device) return null;

  return (
    <Modal
      opened={opened}
      onClose={onClose}
      title="Assign Device"
      centered
    >
      <form onSubmit={form.onSubmit(handleSubmit)}>
        <Stack>
          <div>
            <Text size="sm" c="dimmed">Device</Text>
            <Text fw={500}>{device.name}</Text>
            {device.owner && (
              <Text size="xs" c="dimmed">Currently owned by: {device.owner.name}</Text>
            )}
          </div>

          <Select
            label="Assign to User"
            placeholder={loadingUsers ? "Loading users..." : "Select a user"}
            data={users.map(user => ({
              value: user.id,
              label: `${user.name} (${user.email})`,
            }))}
            searchable
            disabled={loadingUsers}
            required
            {...form.getInputProps('userId')}
            onChange={(value) => {
              form.setFieldValue('userId', value || '');
              if (value) {
                handleUserSelect(value);
              }
            }}
          />

          <NumberInput
            label="Next Jump Number"
            description={selectedUser 
              ? `This user's current jump count is ${(selectedUser.nextJumpNumber || 1) - 1}`
              : "Starting jump number for this device"
            }
            min={1}
            required
            {...form.getInputProps('nextJumpNumber')}
          />

          <Group justify="flex-end" mt="md">
            <Button variant="subtle" onClick={onClose}>
              Cancel
            </Button>
            <Button
              type="submit"
              loading={loading}
              disabled={loading || loadingUsers}
            >
              Assign Device
            </Button>
          </Group>
        </Stack>
      </form>
    </Modal>
  );
}