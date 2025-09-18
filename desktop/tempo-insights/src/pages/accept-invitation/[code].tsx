// pages/accept-invitation/[code].tsx
import { useState, useEffect } from 'react';
import { useRouter } from 'next/router';
import {
  Container,
  Paper,
  Title,
  Text,
  TextInput,
  PasswordInput,
  Button,
  Stack,
  Alert,
  Center,
  Loader,
} from '@mantine/core';
import { useForm } from '@mantine/form';
import { notifications } from '@mantine/notifications';
import { IconAlertCircle, IconUserCheck } from '@tabler/icons-react';

interface InvitationDetails {
  id: string;
  code: string;
  invitedBy: string;
  proxyUser?: {
    name: string;
    slug: string;
  };
  expiresAt: string;
}

export default function AcceptInvitationPage() {
  const router = useRouter();
  const { code } = router.query;
  const [loading, setLoading] = useState(true);
  const [submitting, setSubmitting] = useState(false);
  const [invitationDetails, setInvitationDetails] = useState<InvitationDetails | null>(null);
  const [error, setError] = useState<string | null>(null);

  const form = useForm({
    initialValues: {
      email: '',
      password: '',
      confirmPassword: '',
    },
    validate: {
      email: (value) => (/^\S+@\S+$/.test(value) ? null : 'Invalid email'),
      password: (value) => 
        value.length >= 8 ? null : 'Password must be at least 8 characters',
      confirmPassword: (value, values) =>
        value === values.password ? null : 'Passwords do not match',
    },
  });

  useEffect(() => {
    if (code) {
      checkInvitation();
    }
  }, [code]);

  const checkInvitation = async () => {
    setLoading(true);
    try {
      const response = await fetch(`/api/invitations/${code}/check`);
      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Invalid invitation');
      }

      setInvitationDetails(data.invitation);
      setError(null);
    } catch (error) {
      setError(error instanceof Error ? error.message : 'Invalid or expired invitation');
      setInvitationDetails(null);
    } finally {
      setLoading(false);
    }
  };

  const handleSubmit = async (values: typeof form.values) => {
    setSubmitting(true);
    try {
      const response = await fetch(`/api/invitations/${code}/claim`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email: values.email,
          password: values.password,
        }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Failed to claim account');
      }

      notifications.show({
        title: 'Success!',
        message: 'Your account has been created. Redirecting to login...',
        color: 'green',
        icon: <IconUserCheck />,
      });

      // Redirect to login after a short delay
      setTimeout(() => {
        router.push('/login');
      }, 2000);
    } catch (error) {
      notifications.show({
        title: 'Error',
        message: error instanceof Error ? error.message : 'Failed to claim account',
        color: 'red',
      });
    } finally {
      setSubmitting(false);
    }
  };

  if (loading) {
    return (
      <Container size="xs" py="xl">
        <Center style={{ minHeight: '400px' }}>
          <Stack align="center">
            <Loader size="lg" />
            <Text c="dimmed">Checking invitation...</Text>
          </Stack>
        </Center>
      </Container>
    );
  }

  if (error || !invitationDetails) {
    return (
      <Container size="xs" py="xl">
        <Paper p="xl" withBorder>
          <Stack>
            <Alert icon={<IconAlertCircle size={16} />} color="red" title="Invalid Invitation">
              {error || 'This invitation is invalid or has expired.'}
            </Alert>
            <Button variant="subtle" onClick={() => router.push('/login')}>
              Go to Login
            </Button>
          </Stack>
        </Paper>
      </Container>
    );
  }

  return (
    <Container size="xs" py="xl">
      <Paper p="xl" withBorder>
        <Stack gap="lg">
          <div>
            <Title order={2} mb="sm">Claim Your Account</Title>
            {invitationDetails.proxyUser && (
              <Text c="dimmed">
                Welcome {invitationDetails.proxyUser.name}! Set up your login credentials to access your jump data.
              </Text>
            )}
            <Text size="sm" c="dimmed" mt="xs">
              Invited by: {invitationDetails.invitedBy}
            </Text>
          </div>

          <form onSubmit={form.onSubmit(handleSubmit)}>
            <Stack gap="md">
              <TextInput
                label="Email Address"
                placeholder="your@email.com"
                required
                {...form.getInputProps('email')}
              />

              <PasswordInput
                label="Password"
                placeholder="At least 8 characters"
                required
                {...form.getInputProps('password')}
              />

              <PasswordInput
                label="Confirm Password"
                placeholder="Enter password again"
                required
                {...form.getInputProps('confirmPassword')}
              />

              <Button type="submit" loading={submitting} fullWidth>
                Create Account
              </Button>
            </Stack>
          </form>

          <Text size="xs" c="dimmed" ta="center">
            This invitation expires on {new Date(invitationDetails.expiresAt).toLocaleDateString()}
          </Text>
        </Stack>
      </Paper>
    </Container>
  );
}