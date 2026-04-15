import argparse
import os
import numpy as np
import pandas as pd
from scipy.special import comb

def bezier_to_monomials(control_pts: np.ndarray) -> np.ndarray:
    n = len(control_pts) -1
    coeffs = np.zeros(n+1)

    for j in range(n+1):
        for i in range(j+1):
            coeffs[j] += (
                (-1) ** (j - i)
                * comb(n, j, exact=True)
                * comb(j, i, exact=True)
                * control_pts[i]
            )
    
    return coeffs

def rescale_to_duration(coeffs: np.ndarray, duration: float) -> np.ndarray:
    T_powers = np.array([max(duration, 1e-6) ** i for i in range(len(coeffs))])
    return coeffs / T_powers

def pad_to_degree7(coeffs: np.ndarray) -> np.ndarray:
    if len(coeffs) >= 8:
        return coeffs[:8]
    return np.pad(coeffs, (0, 8 - len(coeffs)))
 

def convert_piece(control_x, control_y, control_z, control_yaw, duration:float) -> np.ndarray:
    """ return [duration, px0..px7, py0..py7, pz0..pz7, pyaw0..pyaw7] """
    def process(pts):
        c = bezier_to_monomials(pts)
        c = rescale_to_duration(c, duration)
        return pad_to_degree7(c)

    row = np.concatenate([
        [duration],
        process(control_x),
        process(control_y),
        process(control_z),
        process(control_yaw),
    ])

    return row


def convert_agent(agent_df: pd.DataFrame, total_duration, speed: float):
    """ return (N_pieces, 33) array """
    
    header = "duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7," \
             "y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7," \
             "z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7," \
             "yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7"

    rows = []

    num_pieces = agent_df["piece_id"].nunique()
    if total_duration is not None:
        duration_per_piece = float(total_duration) / num_pieces
    else:
        duration_per_piece = 1.0 # Default value if duration not provided

    for piece_id, piece_df in agent_df.groupby("piece_id"):
        piece_df = piece_df.sort_values("point_index_in_piece")
        control_x = piece_df["x"].values.astype(float)
        control_y = piece_df["y"].values.astype(float)
        control_z = piece_df["z"].values.astype(float)
        control_yaw = np.zeros(len(control_x)) 

        dur = duration_per_piece

        row = convert_piece(control_x, control_y, control_z, control_yaw, dur)
        rows.append(row)

    return np.array(rows), header

def main():
    parser = argparse.ArgumentParser(description="Bezier -> Crazyswarm2 trajectory converter")
    parser.add_argument("--input", required=True, help="Input CSV file")
    parser.add_argument("--out_dir", default="./trajectories", help="Output directory")
    parser.add_argument("--duration", type=float, default=None, help="Duration of the trajectory in seconds")
    parser.add_argument("--speed", type=float, default=1.0, help="Avergage speed (m/s)")

    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    
    df = pd.read_csv(args.input)
    df.columns = df.columns.str.strip()

    print(f"Uploaded: {len(df)} line, {df['agent_id'].nunique()} agent")

    for agent_id, agent_df in df.groupby("agent_id"):
        rows, header = convert_agent(agent_df, args.duration, args.speed)
        out_path = os.path.join(args.out_dir, f"agent_{agent_id}.csv")
        np.savetxt(out_path, rows, fmt="%.6f", delimiter=",", header=header, comments="")
        total_dur = rows[:,0].sum()
        print(f"  agent_{agent_id}: {len(rows)} piece, toplam {total_dur:.2f} sn → {out_path}")


if __name__ == "__main__":
    main()
#