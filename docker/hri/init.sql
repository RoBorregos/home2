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
    context text
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