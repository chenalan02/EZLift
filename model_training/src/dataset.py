import os
import random
import shutil
import yaml

def preproc_datasets(datasets, output_path, labels, split_ratios=[0.88, 0.08, 0.04]):
    """
    Combine multiple roboflow datasets in YOLO format into one dataset.

    Args:
        datasets_paths (List[Dict]): List of dictionaries of dataset paths and parameters
            Dict:
                path (str): path to the dataset
                label_changes (Dict): dictionary of class name changes
                exclude_from_val (bool): exclude dataset from validation set
                keep_dupe_filenames (bool): filenames from roboflow datasets have the original filename before .rf. and a random string after
                                            if True, keep all images, if False, remove duplicates of the original filenames (including augmentations)
        output_path (str): path to the output directory
        labels (List[str]): list of class names to keep in the output dataset
        split_ratios (List[float]): list of ratios to split the data into train, test, and validation sets
    """
    random.seed(42)

    if os.path.exists(output_path):
        print(f"Output directory {output_path} already exists")
    else:
        os.chdir("model_training")
        curr_dir = os.getcwd()

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
        dataset_summaries = []

        for dataset in datasets:
            dataset_path = dataset['path']
            # save image names as keys and paths as values to batch copy after processing
            # also used to prevent copying duplicate images
            imgs_to_copy = {}

            print(f"Processing dataset: {dataset_path}")
            with open(os.path.join(dataset_path, 'data.yaml'), 'r') as file:
                data = yaml.safe_load(file)
                dataset_names.append(data['roboflow']['project'])
                dataset_urls.append(data['roboflow']['url'])

                # change class names
                for i, class_name in enumerate(data['names']):
                    if class_name in dataset['label_changes']:
                        data['names'][i] = dataset['label_changes'][class_name]

                # get label index conversions
                label_conv_dict = {}
                for i, class_name in enumerate(data['names']):
                    if class_name in labels:
                        new_idx = labels.index(class_name)
                        label_conv_dict[i] = new_idx

                # values to keep track of processing
                images_processed = 0
                images_copied = 0
                invalid_bb = 0
                duplicates_skipped = 0
                other_labels = 0
                # copy images and new labels to output directory randomized by split
                for folder in ['train', 'test', 'valid']:
                    labels_path = os.path.join(dataset_path, folder, 'labels')

                    if not os.path.exists(labels_path):
                        break

                    # copy label files
                    for file_name in os.listdir(labels_path):
                        images_processed += 1

                        # get original file name
                        if dataset['keep_dupe_filenames']:
                            original_file_name = file_name.split('.txt')[0]
                        else:
                            original_file_name = file_name.split('rf')[0]

                        # check duplicate
                        if original_file_name in imgs_to_copy:
                            duplicates_skipped += 1
                            continue
                        
                        if dataset['exclude_from_val']:
                            data_split = random.choices(['train', 'test'], split_ratios[0:2])[0]
                        else:
                            data_split = random.choices(['train', 'test', 'valid'], split_ratios)[0]

                        # copy labels file with new class indicies
                        new_txt = ""
                        valid_bb = True
                        with open(os.path.join(labels_path, file_name), 'r') as read_file:
                            
                            for line in read_file.readlines():
                                line_split = line.split()
                                class_idx = int(line_split[0])

                                # convert to xywh format if in 11-point format
                                if len(line_split) == 11:
                                    line_split = _convert_11_to_xywh(line_split)

                                # skip image if invalid bounding box
                                elif len(line_split) != 5:
                                    if class_idx in label_conv_dict:
                                        new_txt = ""
                                        invalid_bb += 1
                                        valid_bb = False
                                        break
                                
                                # copy relevant bounding boxes with new class index
                                if class_idx in label_conv_dict:
                                    new_class_idx = label_conv_dict[class_idx]
                                    line_split[0] = str(new_class_idx)
                                    new_txt += ' '.join(line_split) + '\n'

                        if not valid_bb:
                            continue

                        # only copy relevant files
                        elif len(new_txt) > 0 or dataset['keep_empty_imgs']:
                            
                            labels_copy_path = os.path.join(output_path, data_split, 'labels', file_name)
                            with open(labels_copy_path, 'w') as write_file:
                                write_file.write(new_txt)

                            image_path = os.path.join(dataset_path, folder, 'images', file_name.replace('txt', 'jpg'))
                            image_copy_path = os.path.join(output_path, data_split, 'images', file_name.replace('txt', 'jpg'))
                            imgs_to_copy[original_file_name] = [image_path, image_copy_path]

                            images_copied += 1

                        else:
                            other_labels += 1

                print(f"Finished Processing {images_processed} images, {images_copied} to keep, {invalid_bb} invalid bounding boxes, {duplicates_skipped} duplicates, {other_labels} other labels")
                dataset_summaries.append({
                    'images_processed' : images_processed,
                    'images_copied' : images_copied,
                    'invalid_bb' : invalid_bb,
                    'duplicates_skipped' : duplicates_skipped,
                    'other_labels' : other_labels
                })

            for path, copy_path in imgs_to_copy.values():
                shutil.copyfile(path, copy_path)

        # create data.yaml file
        yaml_file = {
            'train' : str(os.path.join(curr_dir, output_path, 'train', 'images')),
            'val' : str(os.path.join(curr_dir, output_path, 'valid', 'images')),
            'test' : str(os.path.join(curr_dir, output_path, 'test', 'images')),
            'nc' : len(labels),
            'names' : labels,
            'sources' : []
        }
        for name, url, summary in zip(dataset_names, dataset_urls, dataset_summaries):
            yaml_file['sources'].append({'name' : name, 'url' : url, 'summary' : summary})

        with open(os.path.join(output_path, 'data.yaml'), 'w') as file:
            yaml.dump(yaml_file, file)

def _convert_11_to_xywh(bbox):
    """
    Convert bounding box from 11-point format to xywh format

    Args:
        bbox (List): list of bounding box coordinates in 11-point format (class_idx, x1, y1, x2, y2, x3, y3, x4, y4, x1, y2)

    Returns:
        List: bbox in xywh format
    """
    cls = bbox[0]

    # Extract coordinates from the given bounding box format
    x1, y1 = float(bbox[1]), float(bbox[2])
    x2, y2 = float(bbox[3]), float(bbox[4])
    x3, y3 = float(bbox[5]), float(bbox[6])
    x4, y4 = float(bbox[7]), float(bbox[8])

    # Calculate the center coordinates (x_center, y_center)
    x_center = (x1 + x2 + x3 + x4) / 4
    y_center = (y1 + y2 + y3 + y4) / 4

    # Calculate the width and height
    width = (max(x1, x2, x3, x4) - min(x1, x2, x3, x4))
    height = (max(y1, y2, y3, y4) - min(y1, y2, y3, y4))

    # Return the bounding box in xywh format (relative values)
    return [cls, str(x_center), str(y_center), str(width), str(height)]

if __name__ == '__main__':

    datasets = []
    datasets.append({
        'path' : r'datasets\loco',
        'label_changes' : {},
        'exclude_from_val': False,
        'keep_dupe_filenames': False,
        'keep_empty_imgs': True
    })
    datasets.append({
        'path' : r'datasets\indoor',
        'label_changes' : {},
        'exclude_from_val': False,
        'keep_dupe_filenames': True,
        'keep_empty_imgs': True
    })
    output_path = r'datasets\pallets_dataset_with_negatives'
    labels = ['pallet']
    split_ratios=[0.86, 0.02, 0.12]

    preproc_datasets(datasets, output_path, labels, split_ratios)

