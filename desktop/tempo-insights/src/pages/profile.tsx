import { useState, useEffect } from 'react';
import { Container, Title, Paper, TextInput, Button, Group, Text, Alert, Stack, Card } from '@mantine/core';
import { useForm } from '@mantine/form';
import { notifications } from '@mantine/notifications';
import { AppLayout } from '../components/AppLayout';
import { AuthGuard } from '../components/AuthGuard';
import { ChangePasswordModal } from '../components/ChangePasswordModal';
import { IconLock } from '@tabler/icons-react';

interface ProfileForm {
  name: string;
  email: string;
}

interface UserProfile {
  id: string;
  email: string;
  name: string;
  slug: string;
  role: string;
  createdAt: string;
  jumpCount: number;
  groupCount: number;
  deviceCount: number;
}

export default function ProfilePage() {
  const [loading, setLoading] = useState(false);
  const [user, setUser] = useState<UserProfile | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [passwordModalOpened, setPasswordModalOpened] = useState(false);

  const form = useForm<ProfileForm>({
    initialValues: {
      name: '',
      email: '',
    },
    validate: {
      name: (value) => {
        if (!value) return 'Name is required';
        if (value.length < 2) return 'Name must be at least 2 characters';
        return null;
      },
      email: (value) => {
        if (!value) return 'Email is required';
        if (!/^\S+@\S+$/.test(value)) return 'Invalid email';
        return null;
      },
    },
  });

  useEffect(() => {
    loadProfile();
  }, []);

  const loadProfile = async () => {
    try {
      const response = await fetch('/api/users/me');
      if (!response.ok) throw new Error('Failed to load profile');

      const data = await response.json();
      setUser(data.user);
      form.setValues({
        name: data.user.name,
        email: data.user.email,
      });
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to load profile');
    }
  };

  const handleSubmit = async (values: ProfileForm) => {
    setLoading(true);
    setError(null);

    try {
      const response = await fetch('/api/users/me', {
        method: 'PATCH',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(values),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Failed to update profile');
      }

      setUser(data.user);
      notifications.show({
        title: 'Success',
        message: 'Profile updated successfully',
        color: 'green',
      });
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setLoading(false);
    }
  };

  return (
    <AuthGuard>
      <AppLayout>
        <Container size="sm">
          <Title order={2} mb="xl">My Profile</Title>

          <Stack gap="lg">
            {/* Stats Cards - keep existing */}

            {/* Profile Form */}
            <Paper p="xl" withBorder>
              {/* ... keep existing content ... */}

              <form onSubmit={form.onSubmit(handleSubmit)}>
                <TextInput
                  label="Name"
                  placeholder="Your name"
                  required
                  mb="md"
                  {...form.getInputProps('name')}
                />

                <TextInput
                  label="Email"
                  placeholder="your@email.com"
                  required
                  mb="md"
                  {...form.getInputProps('email')}
                />

                <Group justify="space-between" mt="xl">
                  <Button
                    variant="subtle"
                    leftSection={<IconLock size={16} />}
                    onClick={() => setPasswordModalOpened(true)}
                  >
                    Change Password
                  </Button>

                  <Group>
                    <Button
                      variant="subtle"
                      onClick={() => loadProfile()}
                      disabled={loading}
                    >
                      Cancel
                    </Button>
                    <Button
                      type="submit"
                      loading={loading}
                      disabled={loading}
                      color="brand"
                    >
                      Save Changes
                    </Button>
                  </Group>
                </Group>
              </form>
            </Paper>
          </Stack>
        </Container>

        <ChangePasswordModal
          opened={passwordModalOpened}
          onClose={() => setPasswordModalOpened(false)}
        />
      </AppLayout>
    </AuthGuard>
  );
}
