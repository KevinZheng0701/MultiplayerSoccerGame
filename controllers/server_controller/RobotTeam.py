import threading


class Team:
    """
    Represents a team in a multiplayer game, managing players, scoring, and basic team info.
    """

    def __init__(self, team_number=0, capacity=0):
        """
        Initialize a new team.

        Args:
            team_number (int): Identifier for the team.
            capacity (int): Maximum number of players allowed in the team.
        """
        self.players = set()
        self.team_lock = threading.Lock()
        self.capacity = capacity
        self.team_number = team_number
        self.score = 0

    def __len__(self):
        """
        Return the number of players currently on the team.

        Returns:
            int: Number of players in the team.
        """
        return len(self.players)

    def add_player(self, player):
        """
        Add a player to the team if capacity allows.

        Args:
            player (str): The player ID to add.
        """
        with self.team_lock:
            if len(self.players) < self.capacity:
                self.players.add(player)

    def remove_player(self, player):
        """
        Remove a player from the team.

        Args:
            player (str): The player ID to remove.
        """
        with self.team_lock:
            self.players.discard(player)

    def get_players(self):
        """
        Get all player IDs in the team.

        Returns:
            list[str]: List of player IDs.
        """
        return list(self.players)

    def get_team_number(self):
        """
        Get the team's identifier number.

        Returns:
            int: The team number.
        """
        return self.team_number

    def has(self, player_id):
        """
        Check if a player is part of the team.

        Args:
            player_id (str): The player ID to check.

        Returns:
            bool: True if the player is in the team, False otherwise.
        """
        return player_id in self.players

    def add_score(self):
        """
        Increment the team's score by one.
        """
        self.score += 1

    def get_score(self):
        """
        Get the current score of the team.

        Returns:
            int: The current team score.
        """
        return self.score
