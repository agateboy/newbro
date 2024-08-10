import streamlit as st
from sqlalchemy import create_engine
import pandas as pd
import base64
from PIL import Image
import io
import time
st.set_page_config(layout='wide')
import pydeck as pdk
def create_connection():
    engine = create_engine('mysql+mysqlconnector://mavis:!23Mavis@192.168.18.252/mavis')
    return engine

def fetch_latest_row(connection):
    query = "SELECT * FROM gambar ORDER BY timestamp DESC LIMIT 1"
    df = pd.read_sql(query, connection)
    return df

def base64_to_image(base64_string):
    image = base64.b64decode(base64_string)
    image = Image.open(io.BytesIO(image))
    return image

def fetch_latest_row_red(connection):
    query = "SELECT * FROM gambarmerah ORDER BY timestamp DESC LIMIT 1"
    df = pd.read_sql(query, connection)
    return df

def base64_to_image_red(base64_string):
    image = base64.b64decode(base64_string)
    image = Image.open(io.BytesIO(image))
    return image

def fetch_latest_row_position(connection):
    query = "SELECT * FROM position ORDER BY timestamp DESC LIMIT 1"
    df = pd.read_sql(query, connection)
    return df

def base64_to_image_position(base64_string):
    image = base64.b64decode(base64_string)
    image = Image.open(io.BytesIO(image))
    return image
def fetch_all_position_data(connection):
    query = "SELECT * FROM position"
    df = pd.read_sql(query, connection)
    return df


def main():
    col1, col2, col3 = st.columns(3)
    col1a, col1b = st.columns(2)
    with col1:
        st.title("Cam and GPS Monitoring")
        image_placeholder_position = st.empty()
        latitude_placeholder_position = st.empty()
        longitude_placeholder_position = st.empty()
        sog_placeholder_position = st.empty()
        cog_placeholder_position = st.empty()
    with col2:
        map_placeholder_position = st.empty()

    with col3:
        with col1a:
            st.title("Green Box Monitoring")
            image_placeholder_gambar = st.empty()
            latitude_placeholder = st.empty()
            longitude_placeholder = st.empty()
            sog_placeholder = st.empty()
            cog_placeholder = st.empty()

        with col1b:
            st.title("Red Box Monitoring")
            image_placeholder_merah = st.empty()
            latitude_placeholder_merah = st.empty()
            longitude_placeholder_merah = st.empty()
            sog_placeholder_merah = st.empty()
            cog_placeholder_merah = st.empty()

    connection = create_connection()
    previous_data = None
    while True:
        latest_data = fetch_all_position_data(connection)
        if previous_data is None or not latest_data.equals(previous_data):
            scatter_layer = pdk.Layer(
                "ScatterplotLayer",
                data=latest_data,
                get_position=['longitude', 'latitude'],
                get_radius=10,
                get_fill_color=[255, 0, 0], 
                pickable=True,
            )
            map_position = pdk.Deck(
                map_style='mapbox://styles/mapbox/light-v9',
                initial_view_state=pdk.ViewState(
                    latitude=latest_data['latitude'].mean(),
                    longitude=latest_data['longitude'].mean(),
                    zoom=17,
                    pitch=0,
                ),
                layers=[scatter_layer]
            )
            map_placeholder_position.pydeck_chart(map_position)

            previous_data = latest_data
        latest_row = fetch_latest_row(connection)
        latest_row_red = fetch_latest_row_red(connection)
        latest_row_position = fetch_latest_row_position(connection)

        base64_image_position = latest_row_position['image'].iloc[0]
        latitude_position = latest_row_position['latitude'].iloc[0]
        longitude_position = latest_row_position['longitude'].iloc[0]
        cog_position = latest_row_position['cog'].iloc[0]
        sog_position = latest_row_position['sog'].iloc[0]

        base64_image = latest_row['image'].iloc[0]
        latitude = latest_row['latitude'].iloc[0]
        longitude = latest_row['longitude'].iloc[0]
        cog = latest_row['cog'].iloc[0]
        sog = latest_row['sog'].iloc[0]

        base64_image_red = latest_row_red['image'].iloc[0]
        latitude_red = latest_row_red['latitude'].iloc[0]
        longitude_red = latest_row_red['longitude'].iloc[0]
        cog_red = latest_row_red['cog'].iloc[0]
        sog_red = latest_row_red['sog'].iloc[0]

        jpg_image_position = base64_to_image_position(base64_image_position)

        jpg_image = base64_to_image(base64_image)

        jpg_image_red = base64_to_image_red(base64_image_red)

        image_placeholder_gambar.image(jpg_image, caption='Latest Image (Hijau)', width=200)
        image_placeholder_position.image(jpg_image_position, caption='Latest Image (Position)', width=200)
        image_placeholder_merah.image(jpg_image_red, caption='Latest Image (Merah)', width=200)

        latitude_placeholder.text(f"Latitude (Live): {latitude}")
        latitude_placeholder_position.text(f"Latitude (Position): {latitude_position}")
        latitude_placeholder_merah.text(f"Latitude (Merah): {latitude_red}")

        longitude_placeholder_position.text(f"Longitude (Position): {longitude_position}")
        longitude_placeholder.text(f"Longitude (Gambar): {longitude}")
        longitude_placeholder_merah.text(f"Longitude (Merah): {longitude_red}")

        cog_placeholder_position.text(f"COG (Position): {cog_position}")
        cog_placeholder.text(f"COG (Gambar): {cog}")
        cog_placeholder_merah.text(f"COG (Merah): {cog_red}")

        sog_placeholder_position.text(f"SOG (Position): {sog_position}")
        sog_placeholder.text(f"SOG (Gambar): {sog}")
        sog_placeholder_merah.text(f"SOG (Merah): {sog_red}")

        time.sleep(0.2)

if _name_ == "_main_":
    main()
