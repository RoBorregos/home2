CREATE TABLE items (
    id SERIAL PRIMARY KEY,
    text text NOT NULL,
    embedding vector(384) NOT NULL,
    context text,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE actions (
    id SERIAL PRIMARY KEY,
    action VARCHAR(255) NOT NULL,
    embedding vector(384) NOT NULL
);

CREATE TABLE locations (
    id SERIAL PRIMARY KEY,
    area VARCHAR(255) NOT NULL,
    subarea VARCHAR(255) NOT NULL,
    embedding vector(384) NOT NULL,
    context text,
    context_embedding vector(384) NOT NULL
);

CREATE TABLE command_history (
    id SERIAL PRIMARY KEY,
    action VARCHAR(255) NOT NULL,
    command text NOT NULL,
    result VARCHAR(255) NOT NULL,
    status VARCHAR(255) NOT NULL,
    embedding vector(384) NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE knowledge (
    id SERIAL PRIMARY KEY,
    text text NOT NULL,
    embedding vector(384) NOT NULL,
    context text,
    knowledge_type text NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE hand_location (
    id SERIAL PRIMARY KEY,
    name VARCHAR(255) NOT NULL,
    description text NOT NULL,
    embedding_name vector(384) NOT NULL,
    embedding_description vector(384) NOT NULL,
    x_loc FLOAT NOT NULL,
    y_loc FLOAT NOT NULL,
    m_loc_x FLOAT NOT NULL,
    m_loc_y FLOAT NOT NULL,
    color VARCHAR(50) NOT NULL
);

-- Semantic map: detected objects persisted with a map-frame position.
-- Rows are deduplicated in the adapter (same label within a radius updates
-- the existing row instead of inserting a new one), so this holds one row
-- per physically distinct object the robot has observed, not one per detection.
CREATE TABLE semantic_objects (
    id SERIAL PRIMARY KEY,
    label VARCHAR(255) NOT NULL,
    x FLOAT NOT NULL,
    y FLOAT NOT NULL,
    z FLOAT NOT NULL,
    frame_id VARCHAR(255) NOT NULL DEFAULT 'map',
    confidence FLOAT NOT NULL,
    area VARCHAR(255),
    observations INTEGER NOT NULL DEFAULT 1,
    first_seen TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    last_seen TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_semantic_objects_label ON semantic_objects (label);