import { useState } from 'react';
import { useRouter } from 'next/router';
import {
  Container,
  Paper,
  TextInput,
  PasswordInput,
  Button,
  Title,
  Text,
  Anchor,
  Alert,
} from '@mantine/core';
import { useForm } from '@mantine/form';
import { notifications } from '@mantine/notifications';
import Link from 'next/link';

interface LoginForm {
  email: string;
  password: string;
}

export default function LoginPage() {
  const router = useRouter();
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const form = useForm<LoginForm>({
    initialValues: {
      email: '',
      password: '',
    },
    validate: {
      email: (value) => {
        if (!value) return 'Email is required';
        if (!/^\S+@\S+$/.test(value)) return 'Invalid email';
        return null;
      },
      password: (value) => {
        if (!value) return 'Password is required';
        return null;
      },
    },
  });

  const handleSubmit = async (values: LoginForm) => {
    setLoading(true);
    setError(null);

    try {
      const response = await fetch('/api/auth/login', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(values),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Login failed');
      }

      notifications.show({
        title: 'Success',
        message: 'Logged in successfully!',
        color: 'green',
      });

      // Redirect to home page
      router.push('/home');
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setLoading(false);
    }
  };

  return (
    <Container size={420} my={100}>
      <Title order={1} ta="center" mb={30} c="accent.6">
        Tempo Insights
      </Title>

      <Paper withBorder shadow="md" p={40} radius="md">
        <Title order={2} ta="center" mb="md">
          Sign In
        </Title>

        <form onSubmit={form.onSubmit(handleSubmit)}>
          {error && (
            <Alert color="red" mb="md">
              {error}
            </Alert>
          )}

          <TextInput
            label="Email"
            placeholder="your@email.com"
            required
            mb="md"
            {...form.getInputProps('email')}
          />

          <PasswordInput
            label="Password"
            placeholder="Your password"
            required
            mb="md"
            {...form.getInputProps('password')}
          />

          <Button
            fullWidth
            type="submit"
            loading={loading}
            disabled={loading}
            color="brand"
            mb="md"
          >
            Sign In
          </Button>
        </form>

        <Text ta="center" size="sm">
          Don&apos;t have an account?{' '}
          <Anchor component={Link} href="/register" c="accent.6">
            Create one
          </Anchor>
        </Text>
      </Paper>
    </Container>
  );
}
