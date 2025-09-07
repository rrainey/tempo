import { Modal, TextInput, Textarea, Switch, Button, Stack } from '@mantine/core';
import { useForm } from '@mantine/form';
import { notifications } from '@mantine/notifications';
import { useState } from 'react';
import { useRouter } from 'next/router';

interface CreateGroupForm {
  name: string;
  description: string;
  isPublic: boolean;
}

interface CreateGroupModalProps {
  opened: boolean;
  onClose: () => void;
  onSuccess?: () => void;
}

export function CreateGroupModal({ opened, onClose, onSuccess }: CreateGroupModalProps) {
  const router = useRouter();
  const [loading, setLoading] = useState(false);

  const form = useForm<CreateGroupForm>({
    initialValues: {
      name: '',
      description: '',
      isPublic: true,
    },
    validate: {
      name: (value) => {
        if (!value) return 'Group name is required';
        if (value.length > 100) return 'Name must be less than 100 characters';
        return null;
      },
      description: (value) => {
        if (value && value.length > 500) return 'Description must be less than 500 characters';
        return null;
      },
    },
  });

  const handleSubmit = async (values: CreateGroupForm) => {
    setLoading(true);

    try {
      const response = await fetch('/api/groups', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(values),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Failed to create group');
      }

      notifications.show({
        title: 'Success',
        message: 'Group created successfully!',
        color: 'green',
      });

      form.reset();
      onClose();
      onSuccess?.();

      // Navigate to the new group
      router.push(`/groups/${data.group.slug}`);
    } catch (error) {
      notifications.show({
        title: 'Error',
        message: error instanceof Error ? error.message : 'Failed to create group',
        color: 'red',
      });
    } finally {
      setLoading(false);
    }
  };

  return (
    <Modal
      opened={opened}
      onClose={onClose}
      title="Create New Group"
      centered
      size="md"
    >
      <form onSubmit={form.onSubmit(handleSubmit)}>
        <Stack>
          <TextInput
            label="Group Name"
            placeholder="e.g., Weekend Warriors"
            required
            {...form.getInputProps('name')}
          />

          <Textarea
            label="Description"
            placeholder="What's this group about?"
            rows={3}
            {...form.getInputProps('description')}
          />

          <Switch
            label="Public Group"
            description="Public groups can be discovered and joined by anyone"
            checked={form.values.isPublic}
            {...form.getInputProps('isPublic', { type: 'checkbox' })}
          />

          <Button
            type="submit"
            loading={loading}
            disabled={loading}
            fullWidth
            mt="md"
          >
            Create Group
          </Button>
        </Stack>
      </form>
    </Modal>
  );
}
