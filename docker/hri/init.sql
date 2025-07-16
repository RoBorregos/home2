CREATE TABLE items (
    id SERIAL PRIMARY KEY,
    text text NOT NULL,
    embedding vector(384) NOT NULL,
    context text,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    text_vector tsvector
);

CREATE TABLE actions (
    id SERIAL PRIMARY KEY,
    action VARCHAR(255) NOT NULL,
    embedding vector(384) NOT NULL,
    text_vector tsvector
);

CREATE TABLE locations (
    id SERIAL PRIMARY KEY,
    area VARCHAR(255) NOT NULL,
    subarea VARCHAR(255) NOT NULL,
    embedding vector(384) NOT NULL,
    context text,
    context_embedding vector(384) NOT NULL,
    text_vector tsvector
);

CREATE TABLE command_history (
    id SERIAL PRIMARY KEY,
    action VARCHAR(255) NOT NULL,
    command text NOT NULL,
    result VARCHAR(255) NOT NULL,
    status VARCHAR(255) NOT NULL,
    embedding vector(384) NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    text_vector tsvector
);

CREATE TABLE knowledge (
    id SERIAL PRIMARY KEY,
    text text NOT NULL,
    embedding vector(384) NOT NULL,
    context text,
    knowledge_type text NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    text_vector tsvector
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
    color VARCHAR(50) NOT NULL,
    text_vector tsvector
);


-- For items
CREATE FUNCTION items_text_vector_trigger() RETURNS trigger AS $$
BEGIN
  NEW.text_vector :=
    to_tsvector('english', coalesce(NEW.text, ''));
  RETURN NEW;
END
$$ LANGUAGE plpgsql;

-- For actions
CREATE FUNCTION actions_text_vector_trigger() RETURNS trigger AS $$
BEGIN
  NEW.text_vector :=
    to_tsvector('english', coalesce(NEW.action, ''));
  RETURN NEW;
END
$$ LANGUAGE plpgsql;

-- For locations (concat area + subarea)
CREATE FUNCTION locations_text_vector_trigger() RETURNS trigger AS $$
BEGIN
  NEW.text_vector :=
    to_tsvector('english', coalesce(NEW.area, '') || ' ' || coalesce(NEW.subarea, ''));
  RETURN NEW;
END
$$ LANGUAGE plpgsql;

-- For command_history
CREATE FUNCTION command_history_text_vector_trigger() RETURNS trigger AS $$
BEGIN
  NEW.text_vector :=
    to_tsvector('english', coalesce(NEW.command, ''));
  RETURN NEW;
END
$$ LANGUAGE plpgsql;

-- For knowledge
CREATE FUNCTION knowledge_text_vector_trigger() RETURNS trigger AS $$
BEGIN
  NEW.text_vector :=
    to_tsvector('english', coalesce(NEW.text, ''));
  RETURN NEW;
END
$$ LANGUAGE plpgsql;

-- For hand_location (concat name + description)
CREATE FUNCTION hand_location_text_vector_trigger() RETURNS trigger AS $$
BEGIN
  NEW.text_vector :=
    to_tsvector('english', coalesce(NEW.name, '') || ' ' || coalesce(NEW.description, ''));
  RETURN NEW;
END
$$ LANGUAGE plpgsql;

CREATE TRIGGER trg_items_text_vector
BEFORE INSERT OR UPDATE ON items
FOR EACH ROW EXECUTE FUNCTION items_text_vector_trigger();

CREATE TRIGGER trg_actions_text_vector
BEFORE INSERT OR UPDATE ON actions
FOR EACH ROW EXECUTE FUNCTION actions_text_vector_trigger();

CREATE TRIGGER trg_locations_text_vector
BEFORE INSERT OR UPDATE ON locations
FOR EACH ROW EXECUTE FUNCTION locations_text_vector_trigger();

CREATE TRIGGER trg_command_history_text_vector
BEFORE INSERT OR UPDATE ON command_history
FOR EACH ROW EXECUTE FUNCTION command_history_text_vector_trigger();

CREATE TRIGGER trg_knowledge_text_vector
BEFORE INSERT OR UPDATE ON knowledge
FOR EACH ROW EXECUTE FUNCTION knowledge_text_vector_trigger();

CREATE TRIGGER trg_hand_location_text_vector
BEFORE INSERT OR UPDATE ON hand_location
FOR EACH ROW EXECUTE FUNCTION hand_location_text_vector_trigger();
