"""
Server pool for RL training optimization.

Maintains a pool of pre-initialized GameServer instances that can be
reused across environment resets to avoid expensive recreation.
"""
from typing import List, Optional, Type
from threading import Lock

from gamegine.representation.game import Game
from gamegine.simulation.GameServer import DiscreteGameServer, ServerConfig


class GameServerPool:
    """
    Pool of reusable GameServer instances.
    
    Reduces overhead from creating new servers on each environment reset
    by reusing existing servers with soft_reset().
    """
    
    def __init__(
        self,
        game: Game,
        server_class: Type[DiscreteGameServer] = DiscreteGameServer,
        config: Optional[ServerConfig] = None,
        pool_size: int = 4,
    ):
        """
        Initialize the server pool.
        
        :param game: The game configuration to use for servers.
        :param server_class: Class of server to create (default DiscreteGameServer).
        :param config: Server configuration (optional).
        :param pool_size: Maximum servers to keep in pool.
        """
        self.game = game
        self.server_class = server_class
        self.config = config or ServerConfig()
        self.pool_size = pool_size
        
        self._pool: List[DiscreteGameServer] = []
        self._lock = Lock()
        self._created = 0
        self._acquired = 0
        self._released = 0
    
    def _create_server(self) -> DiscreteGameServer:
        """Create a new server instance."""
        server = self.server_class(self.config)
        server.load_from_game(self.game)
        self._created += 1
        return server
    
    def acquire(self) -> DiscreteGameServer:
        """
        Get a server from the pool (or create new if empty).
        
        The server will be soft-reset before returning.
        
        :return: A ready-to-use GameServer.
        """
        with self._lock:
            if self._pool:
                server = self._pool.pop()
                server.soft_reset()
            else:
                server = self._create_server()
            self._acquired += 1
            return server
    
    def release(self, server: DiscreteGameServer) -> None:
        """
        Return a server to the pool.
        
        If the pool is full, the server will be discarded.
        
        :param server: The server to return.
        """
        with self._lock:
            if len(self._pool) < self.pool_size:
                self._pool.append(server)
            self._released += 1
    
    def get_stats(self) -> dict:
        """Get pool statistics."""
        with self._lock:
            return {
                "pool_size": len(self._pool),
                "max_size": self.pool_size,
                "created": self._created,
                "acquired": self._acquired,
                "released": self._released,
            }
    
    def clear(self) -> None:
        """Clear all servers from the pool."""
        with self._lock:
            self._pool.clear()
