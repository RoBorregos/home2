# ROS Messages Display Components

This directory contains a modular React component system for displaying ROS2 messages with real-time WebSocket communication.

## File Structure

### Main Component
- **`ros-messages.tsx`** - Main container component that orchestrates all other components

### UI Components
- **`AudioStateIndicator.tsx`** - Displays the current audio state (idle, listening, speaking)
- **`MapModal.tsx`** - Modal component for displaying interactive maps with markers
- **`QuestionModal.tsx`** - Modal component for displaying questions and collecting user answers
- **`MessageItem.tsx`** - Individual message display component with type-specific styling
- **`CurrentMessageOverlay.tsx`** - Overlay component for showing current message during listening state

### Hooks
- **`useWebSocket.ts`** - Custom hook for managing WebSocket connections and message handling

### Types
- **`types.ts`** - TypeScript type definitions and Zod schemas

### Utilities
- **`index.ts`** - Barrel export file for easy imports
- **`video.tsx`** - Video streaming component (existing)

## Component Responsibilities

### RosMessagesDisplay (Main Component)
- Manages overall state (messages, audio state, modals)
- Handles WebSocket communication via useWebSocket hook
- Coordinates between all child components
- Provides the main UI layout

### AudioStateIndicator
- Shows current audio state with appropriate icons and styling
- Displays listening state with animated microphone overlay
- Handles idle and speaking states with different visual indicators

### MapModal
- Displays interactive maps with markers
- Supports keyboard shortcuts (ESC to close, V to toggle axes, C to change axes color)
- Handles image loading states and error handling
- Shows map coordinates and marker information

### QuestionModal
- Displays questions to the user
- Handles answer input with keyboard shortcuts (Enter to send, Shift+Enter for new lines)
- Manages modal state and user interactions

### MessageItem
- Renders individual messages with type-specific styling
- Shows timestamps and appropriate icons for each message type
- Supports different message types: heard, spoken, keyword, answer, user_message, text_spoken

### CurrentMessageOverlay
- Shows the current message being processed during listening state
- Displays with large, prominent styling for visibility
- Animated appearance for better user experience

### useWebSocket Hook
- Manages WebSocket connection lifecycle
- Handles different message types from the server
- Provides connection status and message sending capabilities
- Abstracts WebSocket complexity from components

## Usage

```tsx
import { RosMessagesDisplay } from './components';

export default function App() {
  return <RosMessagesDisplay />;
}
```
