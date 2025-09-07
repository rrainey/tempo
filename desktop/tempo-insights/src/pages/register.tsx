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

interface RegisterForm {
  email: string;
  password: string;
  confirmPassword: string;
  name: string;
}

export default function RegisterPage() {
  const router = useRouter();
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const form = useForm<RegisterForm>({
    initialValues: {
      email: '',
      password: '',
      confirmPassword: '',
      name: '',
    },
    validate: {
      email: (value) => {
        if (!value) return 'Email is required';
        if (!/^\S+@\S+$/.test(value)) return 'Invalid email';
        return null;
      },
      password: (value) => {
        if (!value) return 'Password is required';
        if (value.length < 8) return 'Password must be at least 8 characters';
        return null;
      },
      confirmPassword: (value, values) => {
        if (!value) return 'Please confirm your password';
        if (value !== values.password) return 'Passwords do not match';
        return null;
      },
      name: (value) => {
        if (!value) return 'Name is required';
        if (value.length < 2) return 'Name must be at least 2 characters';
        return null;
      },
    },
  });

  const handleSubmit = async (values: RegisterForm) => {
    setLoading(true);
    setError(null);

    try {
      const response = await fetch('/api/auth/register', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email: values.email,
          password: values.password,
          name: values.name,
        }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Registration failed');
      }

      notifications.show({
        title: 'Welcome!',
        message: 'Account created successfully!',
        color: 'green',
      });

      // Auto-login successful, redirect to home
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
          Create Account
        </Title>

        <form onSubmit={form.onSubmit(handleSubmit)}>
          {error && (
            <Alert color="red" mb="md">
              {error}
            </Alert>
          )}

          <TextInput
            label="Name"
            placeholder="John Doe"
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

          <PasswordInput
            label="Password"
            placeholder="At least 8 characters"
            required
            mb="md"
            {...form.getInputProps('password')}
          />

          <PasswordInput
            label="Confirm Password"
            placeholder="Re-enter your password"
            required
            mb="md"
            {...form.getInputProps('confirmPassword')}
          />

          <Button
            fullWidth
            type="submit"
            loading={loading}
            disabled={loading}
            color="brand"
            mb="md"
          >
            Create Account
          </Button>
        </form>

        <Text ta="center" size="sm">
          Already have an account?{' '}
          <Anchor component={Link} href="/login" c="accent.6">
            Sign in
          </Anchor>
        </Text>
      </Paper>
    </Container>
  );
}
