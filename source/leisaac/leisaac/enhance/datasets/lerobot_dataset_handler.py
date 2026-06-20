import copy

from isaaclab.utils import configclass
from isaaclab.utils.datasets.dataset_file_handler_base import DatasetFileHandlerBase
from isaaclab.utils.datasets.episode_data import EpisodeData

try:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    _HAS_LEROBOT = True
except ImportError:
    _HAS_LEROBOT = False
    LeRobotDataset = None  # Placeholder


@configclass
class LeRobotDatasetCfg:
    """Configuration for the LeRobotDataset."""

    repo_id: str = None
    """Lerobot Dataset repository ID."""
    fps: int = 30
    """Lerobot Dataset frames per second."""
    robot_type: str = "so101_follower"
    """Robot type: so101_follower or bi_so101_follower, etc."""
    features: dict = None
    """Features for the LeRobotDataset."""
    action_align: bool = False
    """Whether the action shape equals to the joint number. If action align, we will convert action to lerobot limit range."""


class LeRobotDatasetHandler(DatasetFileHandlerBase):
    def __init__(self, cfg: LeRobotDatasetCfg):
        self._cfg = copy.deepcopy(cfg)
        self._lerobot_dataset = None
        self._demo_count = 0
        self._env_args = {}

    def create(self, file_path: str, env_name: str = None, resume: bool = False):
        if _HAS_LEROBOT:
            # Original LeRobot logic
            if resume:
                self._lerobot_dataset = LeRobotDataset(repo_id=self._cfg.repo_id)
            else:
                self._lerobot_dataset = LeRobotDataset.create(
                    repo_id=self._cfg.repo_id,
                    fps=self._cfg.fps,
                    robot_type=self._cfg.robot_type,
                    features=self._cfg.features,
                )
        else:
            # Decoupled logic: Use a simple dictionary logger
            print("LeRobot not found. Using GenericDataRecorder.")
            self._lerobot_dataset = GenericDataRecorder(self._cfg.repo_id, self._cfg.fps, self._cfg.features)

    def open(self, file_path: str, mode: str = "r"):
        """Opens an existing dataset for reading or appending."""
        if _HAS_LEROBOT:
            # Standard LeRobot initialization
            self._lerobot_dataset = LeRobotDataset(
                repo_id=self._cfg.repo_id,
            )
        else:
            # Decoupled logic: Initialize your custom handler
            # You can use file_path here to load existing metadata
            self._lerobot_dataset = GenericDataRecorder(
                repo_id=self._cfg.repo_id, fps=self._cfg.fps, features=self._cfg.features
            )
            if mode == "r":
                self._lerobot_dataset.load_from_disk(file_path)

    def get_env_name(self) -> str | None:
        return self._env_args["env_name"]

    def add_frame(self, frame: dict):
        self._lerobot_dataset.add_frame(frame=frame)

    def flush(self):
        if not self.has_episode_buffer():
            return False
        self._lerobot_dataset.save_episode(parallel_encoding=False)
        return True

    def clear(self):
        if not self.has_episode_buffer():
            return False
        self._lerobot_dataset.clear_episode_buffer()
        return True

    def finalize(self):
        self._lerobot_dataset.finalize()

    def has_episode_buffer(self) -> bool:
        episode_buffer = getattr(self._lerobot_dataset, "episode_buffer", None)
        if isinstance(episode_buffer, dict):
            return episode_buffer.get("size", 0) > 0
        if episode_buffer is not None:
            return True
        return bool(getattr(self._lerobot_dataset, "buffer", []))

    def close(self):
        if self._lerobot_dataset is not None:
            self.finalize()
            self._lerobot_dataset = None

    # not used for now
    def write_episode(self, episode: EpisodeData):
        raise NotImplementedError("write_episode is not supported for LeRobotDatasetHandler")

    def load_episode(self, episode_name: str) -> EpisodeData | None:
        raise NotImplementedError("load_episode is not supported for LeRobotDatasetHandler")

    def get_num_episodes(self) -> int:
        if self._lerobot_dataset is None:
            return 0

        num_episodes = getattr(self._lerobot_dataset, "num_episodes", None)
        if num_episodes is not None:
            return int(num_episodes() if callable(num_episodes) else num_episodes)

        metadata = getattr(self._lerobot_dataset, "meta", None)
        if metadata is not None:
            total_episodes = getattr(metadata, "total_episodes", None)
            if total_episodes is not None:
                return int(total_episodes() if callable(total_episodes) else total_episodes)

        info = getattr(self._lerobot_dataset, "info", None)
        if isinstance(info, dict) and "total_episodes" in info:
            return int(info["total_episodes"])

        episode_data_index = getattr(self._lerobot_dataset, "episode_data_index", None)
        if episode_data_index is not None and "from" in episode_data_index:
            return len(episode_data_index["from"])

        return self._demo_count


# Create a simple fallback recorder if LeRobot is missing
class GenericDataRecorder:
    def __init__(self, repo_id, fps, features):
        self.repo_id = repo_id
        self.fps = fps
        self.features = features
        self.buffer = []
        self._demo_count = 0

    def add_frame(self, frame):
        # Just store in memory or append to a list
        self.buffer.append(frame)

    def save_episode(self, **kwargs):
        # Save to a standard format like pickle or numpy
        import pickle

        with open(f"{self.repo_id}_episode.pkl", "wb") as f:
            pickle.dump(self.buffer, f)
        self.buffer = []
        self._demo_count += 1

    def finalize(self):
        print("Dataset finalized.")

    def get_num_episodes(self) -> int:
        return self._demo_count

    def load_from_disk(self, path):
        # Implementation to load your custom .npz or .json files
        # This ensures that even without LeRobot, your 'open' method
        # populates the metadata correctly.
        pass
