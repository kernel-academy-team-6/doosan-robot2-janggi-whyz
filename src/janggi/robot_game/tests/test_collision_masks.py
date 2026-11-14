"""Collision groups keep grasp contacts but stop board-wide island chaining."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from collision_masks import (ARM_CATEGORY, ARM_COLLIDE, ENVIRONMENT_CATEGORY, ENVIRONMENT_COLLIDE,
                             PIECE_CATEGORY, PIECE_COLLIDE, collides)
from piece_visuals import piece_physics

DEFAULT = (0xffff, 0xffff)  # gripper / link_6: no filter
PIECE = (PIECE_CATEGORY, PIECE_COLLIDE)
ENV = (ENVIRONMENT_CATEGORY, ENVIRONMENT_COLLIDE)
ARM = (ARM_CATEGORY, ARM_COLLIDE)


def test_filter_table_matches_documented_design():
    assert collides(*PIECE, *DEFAULT)        # fingers grasp pieces
    assert collides(*PIECE, *ENV)            # pieces rest on board/table
    assert not collides(*PIECE, *PIECE)      # no board-wide AABB island chain
    assert not collides(*PIECE, *ARM)        # upper/lower arm never pairs with pieces
    assert collides(*ARM, *ENV)              # arm still stops on board/table
    assert collides(*DEFAULT, *ENV)


def test_piece_sdf_carries_filter_groups_on_every_collision():
    root = ET.fromstring('<root>'+piece_physics('cha')+'</root>')
    for collision in root.findall('collision'):
        assert int(collision.findtext('surface/contact/category_bitmask')) == PIECE_CATEGORY
        assert int(collision.findtext('surface/contact/collide_bitmask')) == PIECE_COLLIDE
