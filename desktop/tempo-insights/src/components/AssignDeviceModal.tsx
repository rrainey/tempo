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
  onSuccess?: () => void;
}

export function AssignDeviceModal({ opened, onClose, device, onSuccess }: AssignDeviceModalProps) {
  const [loading, setLoading] = useState(false);
  const [users, setUsers] = useState<User[]>([]);
  const [loadingUsers, setLoadingUsers] = useState(true);

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
    }
  }, [opened]);

  const loadUsers = async () => {
    try {
      setLoadingUsers(true);
      // TODO: Replace with actual users endpoint
      const response = await fetch('/api/users');
      if (response.ok) {
        const data = await response.json();
        setUsers(data.users || []);
      } else {
        // For now, use mock data
        setUsers([
          { id: 'user1', name: 'Test User', email: 'test@example.com', slug: 'test-user' },
          { id: 'user2', name: 'Another User', email: 'another@example.com', slug: 'another-user' },
        ]);
      }
    } catch (error) {
      // For now, use mock data
      setUsers([
        { id: 'user1', name: 'Test User', email: 'test@example.com', slug: 'test-user' },
        { id: 'user2', name: 'Another User', email: 'another@example.com', slug: 'another-user' },
      ]);
    } finally {
      setLoadingUsers(false);
    }
  };

  const handleSubmit = async (values: AssignDeviceForm) => {
    if (!device) return;

    setLoading(true);
    try {
      // TODO: Implement actual API endpoint
      const response = await fetch(`/api/devices/${device.id}/assign`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(values),
      });

      if (!response.ok) {
        throw new Error('Failed to assign device');
      }

      // For now, just log the action
      console.log('Assign device:', device.id, 'to user:', values.userId, 'with jump number:', values.nextJumpNumber);

      notifications.show({
        title: 'Device Assigned',
        message: `${device.name} has been assigned (action stubbed)`,
        color: 'blue',
      });

      onClose();
      onSuccess?.();
    } catch (error) {
      notifications.show({
        title: 'Error',
        message: 'Failed to assign device (endpoint not implemented)',
        color: 'red',
      });
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
            placeholder="Select a user"
            data={users.map(user => ({
              value: user.id,
              label: `${user.name} (${user.email})`,
            }))}
            searchable
            disabled={loadingUsers}
            required
            {...form.getInputProps('userId')}
          />

          <NumberInput
            label="Next Jump Number"
            description="Starting jump number for this device"
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
              disabled={loading}
            >
              Assign Device
            </Button>
          </Group>
        </Stack>
      </form>
    </Modal>
  );
}
