from dataclasses import dataclass

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("so101_leader_7dof")
@dataclass
class SO101Leader6DofV3Config(TeleoperatorConfig):
    """Configuration for the SO101 Leader 7dof teleoperator."""

    port: str
    use_degrees: bool = False
