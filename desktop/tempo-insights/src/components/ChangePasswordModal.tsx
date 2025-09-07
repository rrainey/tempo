import { Modal, PasswordInput, Button, Stack, Alert } from '@mantine/core';
import { useForm } from '@mantine/form';
import { notifications } from '@mantine/notifications';
import { useState } from 'react';
import { logout } from '../lib/auth/logout';

interface ChangePasswordForm {
  currentPassword: string;
  newPassword: string;
  confirmPassword: string;
}

interface ChangePasswordModalProps {
  opened: boolean;
  onClose: () => void;
}

export function ChangePasswordModal({ opened, onClose }: ChangePasswordModalProps) {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const form = useForm<ChangePasswordForm>({
    initialValues: {
      currentPassword: '',
      newPassword: '',
      confirmPassword: '',
    },
    validate: {
      currentPassword: (value) => {
        if (!value) return 'Current password is required';
        return null;
      },
      newPassword: (value) => {
        if (!value) return 'New password is required';
        if (value.length < 8) return 'Password must be at least 8 characters';
        return null;
      },
      confirmPassword: (value, values) => {
        if (!value) return 'Please confirm your password';
        if (value !== values.newPassword) return 'Passwords do not match';
        return null;
      },
    },
  });

  const handleSubmit = async (values: ChangePasswordForm) => {
    setLoading(true);
    setError(null);

    try {
      const response = await fetch('/api/users/password', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          currentPassword: values.currentPassword,
          newPassword: values.newPassword,
        }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Failed to change password');
      }

      notifications.show({
        title: 'Success',
        message: 'Password changed successfully. Please log in again.',
        color: 'green',
      });

      // Close modal and logout
      onClose();
      setTimeout(() => {
        logout();
      }, 2000);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setLoading(false);
    }
  };

  return (
    <Modal
      opened={opened}
      onClose={onClose}
      title="Change Password"
      centered
    >
      <form onSubmit={form.onSubmit(handleSubmit)}>
        {error && (
          <Alert color="red" mb="md">
            {error}
          </Alert>
        )}

        <Stack>
          <PasswordInput
            label="Current Password"
            placeholder="Enter your current password"
            required
            {...form.getInputProps('currentPassword')}
          />

          <PasswordInput
            label="New Password"
            placeholder="At least 8 characters"
            required
            {...form.getInputProps('newPassword')}
          />

          <PasswordInput
            label="Confirm New Password"
            placeholder="Re-enter your new password"
            required
            {...form.getInputProps('confirmPassword')}
          />

          <Button
            type="submit"
            loading={loading}
            disabled={loading}
            fullWidth
            mt="md"
          >
            Change Password
          </Button>
        </Stack>
      </form>
    </Modal>
  );
}
