import { Modal, TextInput, Select, Button, Stack, Alert } from '@mantine/core';
import { useForm } from '@mantine/form';
import { notifications } from '@mantine/notifications';
import { useState } from 'react';

interface InviteMemberForm {
  email: string;
  groupRole: string;
}

interface InviteMemberModalProps {
  opened: boolean;
  onClose: () => void;
  groupSlug: string;
  userRole: string;
}

export function InviteMemberModal({ opened, onClose, groupSlug, userRole }: InviteMemberModalProps) {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const form = useForm<InviteMemberForm>({
    initialValues: {
      email: '',
      groupRole: 'MEMBER',
    },
    validate: {
      email: (value) => {
        if (!value) return 'Email is required';
        if (!/^\S+@\S+$/.test(value)) return 'Invalid email';
        return null;
      },
    },
  });

  const handleSubmit = async (values: InviteMemberForm) => {
    setLoading(true);
    setError(null);

    try {
      const response = await fetch(`/api/groups/${groupSlug}/invite`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(values),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Failed to send invitation');
      }

      notifications.show({
        title: 'Success',
        message: data.message,
        color: 'green',
      });

      form.reset();
      onClose();
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setLoading(false);
    }
  };

  const roleOptions = userRole === 'OWNER'
    ? [
        { value: 'MEMBER', label: 'Member' },
        { value: 'ADMIN', label: 'Admin' },
      ]
    : [
        { value: 'MEMBER', label: 'Member' },
      ];

  return (
    <Modal
      opened={opened}
      onClose={onClose}
      title="Invite Member"
      centered
    >
      <form onSubmit={form.onSubmit(handleSubmit)}>
        {error && (
          <Alert color="red" mb="md">
            {error}
          </Alert>
        )}

        <Stack>
          <TextInput
            label="Email Address"
            placeholder="user@example.com"
            required
            {...form.getInputProps('email')}
          />

          <Select
            label="Role"
            data={roleOptions}
            {...form.getInputProps('groupRole')}
          />

          <Button
            type="submit"
            loading={loading}
            disabled={loading}
            fullWidth
            mt="md"
          >
            Send Invitation
          </Button>
        </Stack>
      </form>
    </Modal>
  );
}
