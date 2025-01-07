import os
import random
import numpy as np
import yaml

def combine_datasets(datasets, output_path, labels, split_ratios=[0.88, 0.08, 0.04]):
    """
    Combine multiple roboflow datasets in YOLO format into one dataset.

    Args:
        datasets_paths (List[Dict[str, Union(str, Dict[str, str])]]): Dictionary of dataset paths and label changes
        output_path (str): path to the output directory
        labels (List[str]): list of class names to keep in the output dataset
        split_ratios (List[float]): list of ratios to split the data into train, test, and validation sets
    """

    if os.path.exists(output_path):
        print(f"Output directory {output_path} already exists")
    else:
        # create output directory
        os.makedirs(output_path)
        for folder in ['train', 'test', 'valid']:
            os.makedirs(os.path.join(output_path, folder))
            os.makedirs(os.path.join(output_path, folder, 'images'))
            os.makedirs(os.path.join(output_path, folder, 'labels'))
        print(f"Output directory {output_path} created")

        # get class names, idx as dict
        dataset_names = []
        dataset_urls = []

        for dataset_path in datasets_paths:

            with open(os.path.join(dataset_path, 'data.yaml'), 'r') as file:
                data = yaml.safe_load(file)
                dataset_names.append(data['roboflow']['project'])
                dataset_urls.append(data['roboflow']['url'])

                # get label conversions
                label_conv_dict = {}
                for i, class_name in enumerate(data['names']):
                    class_name = class_name.lower()
                    if class_name in labels:
                        new_idx = labels.index(class_name)
                        label_conv_dict[i] = new_idx

                # copy images and new labels to output directory randomized by split
                for folder in ['train', 'test', 'valid']:
                    labels_path = os.path.join(dataset_path, folder, 'labels')

                    # copy label files
                    for file_name in os.listdir(labels_path):
                        data_split = random.choices(['train', 'test', 'valid'], split_ratios)[0]
                        labels_copy_path = os.path.join(output_path, data_split, 'labels', file_name)

                        # copy labels file with new class indicies
                        new_txt = ""
                        with open(os.path.join(labels_path, file_name), 'r') as read_file:
                            
                                for line in read_file.readlines():
                                    line_split = line.split()
                                    class_idx = int(line_split[0])
                                    if class_idx in label_conv_dict:
                                        new_class_idx = label_conv_dict[class_idx]
                                        line_split[0] = str(new_class_idx)
                                        new_txt += ' '.join(line_split) + '\n'

                        # only copy relevant files
                        if len(new_txt) > 0:
                            with open(labels_copy_path, 'w') as write_file:
                                write_file.write(new_txt)

                            # copy image file
                            image_path = os.path.join(dataset_path, folder, 'images', file_name.replace('txt', 'jpg'))
                            image_copy_path = os.path.join(output_path, data_split, 'images', file_name.replace('txt', 'jpg'))

        # create data.yaml file
        yaml_file = {
            'train' : str(os.path.join(output_path, 'train', 'images')),
            'val' : str(os.path.join(output_path, 'valid', 'images')),
            'test' : str(os.path.join(output_path, 'test', 'images')),

            'nc' : len(labels),
            'names' : labels,

            'sources' : []
        }
        for name, url in zip(dataset_names, dataset_urls):
            yaml_file['sources'].append({'name' : name, 'url' : url})

if __name__ == '__main__':

    datasets_paths = []
    datasets_paths.append({
        'path' : 'Dataset\1000ware',
        'label_changes' : {
            'Box' : 'box',
        }
    })
    datasets_paths.append({
        'path' : 'Dataset\industrial_dataset3',
        'label_changes' : {
            'pallet' : 'pallet'
        }
    })
    datasets_paths.append('Dataset\industrial_dataset3')

    output_path = 'Dataset\combined_dataset'
    labels = ['box, pallet']
    split_ratios=[0.88, 0.08, 0.04]

    combine_datasets(datasets_paths, output_path, labels, split_ratios)