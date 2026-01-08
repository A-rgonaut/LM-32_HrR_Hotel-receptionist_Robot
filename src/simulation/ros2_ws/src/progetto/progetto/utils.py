import json

class Entity():
    def __init__(self, id):
        self.id = id

    def __str__(self):
        return json.dumps(self.__dict__, ensure_ascii=False)

    def __eq__(self, other):
        if isinstance(other, Entity):
            return self.id == other.id
        return False

    def __hash__(self):
        return hash(self.id)

    def __repr__(self):
        return f"Entity({self.id!r})"

class Persona(Entity):
    def __init__(self, id, nome, cognome):
        super().__init__(id)
        self.nome = nome
        self.cognome = cognome

    def __repr__(self):
        return f"Persona({self.id!r}, {self.nome!r}, {self.cognome!r})"

class Ospite(Persona):
    def __init__(self, id, nome, cognome, eta, lingua):
        super().__init__(id, nome, cognome)
        self.eta = eta
        self.lingua = lingua

    def __repr__(self):
        return f"Ospite({self.id!r}, {self.nome!r}, {self.cognome!r}, {self.eta!r}, {self.lingua!r})"

class Luogo(Entity):
    def __init__(self, id, x, y):
        super().__init__(id)
        self.x = x
        self.y = y

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __repr__(self):
        return f"{type(self).__name__}({self.id!r}, {self.x!r}, {self.y!r})"

class Hall(Luogo):
    def __init__(self, id, x, y):
        super().__init__(id, x, y)

class Stanza(Luogo):
    def __init__(self, id, x, y, nome):
        super().__init__(id, x, y)
        self.nome = nome

    def __repr__(self):
        return f"Stanza({self.id!r}, {self.x!r}, {self.y!r}, {self.nome!r})"

class CoffeeRoom(Luogo):
    def __init__(self, id, x, y):
        super().__init__(id, x, y)
